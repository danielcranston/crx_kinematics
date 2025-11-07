from dataclasses import dataclass
import io
import time
from typing import List, Union

from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.logging
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations as tr

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


class ReBroadcastableStaticTransformBroadcaster(StaticTransformBroadcaster):
    """
    In ROS 2 Jazzy, StaticTransformBroadcaster does not allow updating an already sent
    static TF. This class provides a workaround. See https://github.com/ros2/geometry2/issues/704.

    While the authorative take in the previous issue is that disallowing updating is correct
    behavior (I personally disagree ^^), a separate PR has since been merged into Rolling that
    goes back to allowing updating. See https://github.com/ros2/geometry2/pull/820.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tf_cache = {}

    def sendTransform(self, transform: Union[TransformStamped, List[TransformStamped]]) -> None:
        if isinstance(transform, TransformStamped):
            transform = [transform]

        for tf in transform:
            self.tf_cache[tf.child_frame_id] = tf

        self.pub_tf.publish(TFMessage(transforms=self.tf_cache.values()))


def matrix_to_transform(matrix: np.ndarray) -> Transform:
    trans = tr.translation_from_matrix(matrix)
    quat = tr.quaternion_from_matrix(matrix)

    return Transform(
        translation=Vector3(x=trans[0], y=trans[1], z=trans[2]),
        rotation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
    )


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    trans = tr.translation_from_matrix(matrix)
    quat = tr.quaternion_from_matrix(matrix)

    return Pose(
        translation=Point(x=trans[0], y=trans[1], z=trans[2]),
        rotation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
    )


def to_xyzwpr(T):
    xyz = (1000 * T[:3, 3]).tolist()
    wpr = np.array([np.degrees(a) for a in tr.euler_from_matrix(T)]).tolist()

    return xyz + wpr


def from_xyzwpr(xyzwpr):
    T = tr.euler_matrix(*[np.radians(a) for a in xyzwpr[3:]])
    T[:3, 3] = [p / 1000 for p in xyzwpr[:3]]

    return T


def numpy_to_point(arr):
    return Point(x=arr[0], y=arr[1], z=arr[2])


def isometry_inv(T):
    """
    Returns the inverse of a 4x4 matrix representation of an isometry (rigid transformation).
    This is essentially a faster version of the more general np.linalg.inv.

    Example:
    >>> T = tr.random_rotation_matrix()
    >>> T[:3, 3] = tr.random_vector(3)
    >>> T_inv = isometry_inv(T)
    >>> np.allclose(T_inv, np.linalg.inv(T))
    True
    """
    R_inv = T[:3, :3].T
    t_inv = -R_inv.dot(T[:3, 3])

    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv


def normalized(arr):
    return arr / np.linalg.norm(arr)


def vector_projection(a, b):
    """
    Returns the vector projection of a onto b.
    https://en.wikipedia.org/wiki/Vector_projection
    """
    return a.dot(b) / b.dot(b) * b


def vector_rejection(a, b):
    """
    Returns the vector rejection of a from b. In other words,
    the orthogonal component of a that is perpendicular to b.
    https://en.wikipedia.org/wiki/Vector_projection
    """
    return a - vector_projection(a, b)


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """
    Calculates the (unsigned) angle between two directional 3D vectors
    >>> angle = angle_between([1,0,0], [0,1,0])
    >>> np.isclose(angle, np.radians(90))
    True
    """
    dot = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    return np.arccos(np.clip(dot, -1, 1))


def construct_plane(dir1, dir2):
    dir1_unit = dir1 / np.linalg.norm(dir1)

    dir2_orthogonal = vector_rejection(dir2, dir1)
    dir2_orthogonal = dir2_orthogonal / np.linalg.norm(dir2_orthogonal)

    dir3 = np.cross(dir1_unit, dir2_orthogonal)

    T = np.eye(4)
    T[:3, :3] = np.array([dir1_unit, dir2_orthogonal, dir3]).T
    return T


def find_third_triangle_corner(AB, AC, BC):
    """
    Given a triangle where
      1. The lengths of each side are known,
      2. The first corner is at (0, 0)
      3. The second corner is at (AB, 0)
    Returns the position (x, y) of the third triangle.
    Note that flipping the sign of y also is a valid solution, hence both solutions are returned

           C = (x, y)
         /   \
      /        \
    A ---------- B = (AB, 0)
    
    https://math.stackexchange.com/a/544025
    """
    x = (AC**2 - BC**2 + AB**2) / (2 * AB)

    x_squared = x**2
    AC_squared = AC**2

    if x_squared > AC_squared:
        y = 0.0
    else:
        y = np.sqrt(AC_squared - x_squared)

    return np.array([x, y]), np.array([x, -y])


"""
delta: offset along previous Z to the common normal
theta: angle about previous z from old x to new x
r (or a): length of the common normal. The radius about previous z
alpha: angle about common normal, from old z axis to new z axis
"""


class CircleEvaluation:
    def __init__(self, q, T_R0_tool, dh_params, O5):
        self.q = q

        r4 = dh_params[3].r
        r5 = dh_params[4].r
        r6 = dh_params[5].r
        a3 = dh_params[2].a

        self.O4 = (T_R0_tool @ [r5 * np.cos(q), r5 * np.sin(q), r6, 1])[:3]
        self.dist_O4 = np.linalg.norm(self.O4)
        self.eq_15_holds = self.dist_O4 <= np.abs(r4) + np.abs(a3)
        # if not self.eq_15_holds:
        #     return

        # Create O0O3O4 plane on which O3 will lie (it passes through [0,0,1])
        self.T_R0_plane = construct_plane(self.O4, np.array([0, 0, 1]))

        # Find O3 in that plane. https://math.stackexchange.com/questions/543961/determine-third-point-of-triangle-when-two-points-and-all-sides-are-known
        O3UP, O3DOWN = find_third_triangle_corner(AB=self.dist_O4, AC=a3, BC=-r4)

        # Express the O3 candidates in the reference frame
        self.O3UP = self.T_R0_plane[:3, :3] @ [*O3UP, 0]
        self.O3DOWN = self.T_R0_plane[:3, :3] @ [*O3DOWN, 0]

        self.Z5fulllength = O5 - self.O4
        self.Z5 = normalized(self.Z5fulllength)
        self.Z4UP = normalized(self.O4 - self.O3UP)
        self.Z4DOWN = normalized(self.O4 - self.O3DOWN)
        self.dot_product_up = self.Z4UP @ self.Z5
        self.dot_product_down = self.Z4DOWN @ self.Z5

    def determine_joint_values(self, dh_params, O5, O6, fk, T_R0_tool):
        J1 = np.atan2(self.O4[1], self.O4[0])
        rclpy.logging.get_logger("try").info(f"J1={np.degrees(J1)}")

        T_L1_L0 = isometry_inv(dh_params[0].T(J1))
        R_L1_L0 = T_L1_L0[:3, :3]
        O_1_3 = R_L1_L0 @ self.O3UP
        J2 = -np.atan2(O_1_3[2], O_1_3[0]) + np.pi / 2
        rclpy.logging.get_logger("try").info(f"J2={np.degrees(J2)}")

        O_1_4 = R_L1_L0 @ self.O4
        J3 = np.atan2(O_1_4[2] - O_1_3[2], O_1_4[0] - O_1_3[0])
        rclpy.logging.get_logger("try").info(f"J3={np.degrees(J3)}")

        O_1_5 = R_L1_L0 @ O5
        O4O5_dir = O_1_5 - O_1_4
        J4 = np.atan2(O4O5_dir[2], -O4O5_dir[1])
        rclpy.logging.get_logger("try").info(f"J4={np.degrees(J4)}")

        T_R0_L4 = fk([np.degrees(j) for j in [J1, J2, J3, J4, 0, 0]], up_to=4)

        O_4_6 = isometry_inv(T_R0_L4) @ [*O6, 1.0]
        J5 = np.atan2(O_4_6[0], -O_4_6[2])
        rclpy.logging.get_logger("try").info(f"J5={np.degrees(J5)}")

        T_R0_L5 = T_R0_L4 @ dh_params[4].T(J5)
        T_L5_tool = isometry_inv(T_R0_L5) @ T_R0_tool

        J6 = np.atan2(-T_L5_tool[2, 0], T_L5_tool[0, 0])
        rclpy.logging.get_logger("try").info(f"J6={np.degrees(J6)}")

        solution = [J1, J2, J3, J4, J5, J6]
        return solution

    def markers(self, frame_id="R0"):
        markers = [
            Marker(
                header=Header(frame_id=frame_id),
                ns="O_points",
                type=Marker.SPHERE,
                pose=Pose(position=numpy_to_point(self.O4)),
                scale=Vector3(x=0.02, y=0.02, z=0.02),
                color=ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.5),
            ),
            Marker(
                header=Header(frame_id="plane"),
                ns="O0O3O4_plane",
                pose=matrix_to_pose(self.T_R0_plane),
                type=Marker.CYLINDER,
                scale=Vector3(x=2 * self.dist_O4, y=2 * self.dist_O4, z=0.001),
                color=ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),
            ),
        ]
        markers.extend(
            [
                Marker(
                    header=Header(frame_id=frame_id),
                    ns="Z4Z5_arrows",
                    type=Marker.ARROW,
                    scale=Vector3(x=0.005, y=0.02, z=0.02),
                    color=ColorRGBA(
                        r=0.0 if close_to_zero else 1.0,
                        g=1.0 if close_to_zero else 0.0,
                        b=0.0,
                        a=0.5,
                    ),
                    points=[numpy_to_point(start), numpy_to_point(end)],
                )
                for start, end, close_to_zero in [
                    (self.O4 + self.Z5fulllength, self.O4, False),
                    (self.O3UP, self.O4, np.abs(self.dot_product_up) < 0.1),
                    (self.O3DOWN, self.O4, np.abs(self.dot_product_down) < 0.1),
                ]
            ]
        )
        markers.extend(
            [
                Marker(
                    header=Header(frame_id=frame_id),
                    ns="O3_candidates",
                    type=Marker.SPHERE,
                    pose=Pose(position=numpy_to_point(O)),
                    scale=Vector3(x=0.02, y=0.02, z=0.02),
                    color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5),
                )
                for O in [self.O3UP, self.O3DOWN]
            ]
        )

        return markers


class CRXRobot:
    @dataclass
    class DHParams:
        a: float = 0.0
        alpha: float = 0.0
        r: float = 0.0
        theta: float = 0.0

        def T(self, joint_value=0):  # in radians
            a, alpha, r, theta = self.a, self.alpha, self.r, self.theta

            theta = joint_value + self.theta

            stheta, ctheta = np.sin(theta), np.cos(theta)
            salpha, calpha = np.sin(alpha), np.cos(alpha)

            T = np.eye(4)  # Assign each row. See Eq 1.
            T[0, :4] = [ctheta, -stheta, 0, a]
            T[1, :4] = [stheta * calpha, ctheta * calpha, -salpha, -r * salpha]
            T[2, :4] = [stheta * salpha, ctheta * salpha, calpha, r * calpha]
            return T

    # Corresponds to each column in Table 2.
    # Note that some rows annoyingly attribute values to i-1 while some attribute to i.
    # The below params attribute all values in each column to i.
    L1 = DHParams()
    L2 = DHParams(alpha=-np.pi / 2, theta=-np.pi / 2)
    L3 = DHParams(a=0.54, alpha=np.pi)
    L4 = DHParams(alpha=-np.pi / 2, r=-0.54)
    L5 = DHParams(alpha=np.pi / 2, r=0.15)
    L6 = DHParams(alpha=-np.pi / 2, r=-0.16)

    dh_params = [L1, L2, L3, L4, L5, L6]

    T_L6_tool = np.diag([1.0, -1, -1, 1])
    frame_names = ["R0", "L1", "L2", "L3", "L4", "L5", "L6", "tool"]

    def __init__(self):
        pass

    def fk(self, joint_values=None, return_individual_transforms=False, up_to=7):
        # Joint values in degrees

        if num_values := len(joint_values) != 6:
            raise RuntimeError(f"Received {len(num_values)} joint values for FK, but expected 6")

        if joint_values is None:
            joint_values = [0] * 6

        joint_values = [np.radians(j) for j in joint_values]  # Express in radians

        joint_values[2] += joint_values[1]  # Account for "Fanuc pecularity" mentioned after Table 2

        T_list = [param.T(j) for j, param in zip(joint_values, self.dh_params)]
        T_list.append(self.T_L6_tool)

        T_R0_tool = T_list[0].copy()
        for i, T in enumerate(T_list[1:up_to]):
            T_R0_tool = T_R0_tool @ T

        if return_individual_transforms:
            return T_R0_tool, T_list

        return T_R0_tool

    def ik(self, xyzwpr):
        T_R0_tool = from_xyzwpr(xyzwpr)
        O6 = T_R0_tool[:3, 3]
        O5 = (T_R0_tool @ [0, 0, self.L6.r, 1])[:3]

        circle_evaluations = [
            CircleEvaluation(q, T_R0_tool, self.dh_params, O5)
            for q in np.linspace(0, 2 * np.pi, 360)
        ]

        sample_signal_up, sample_signal_down = zip(
            *[(ce.dot_product_up, ce.dot_product_down) for ce in circle_evaluations]
        )

        zeros = []
        for i in range(1, len(sample_signal_down)):
            if np.sign(sample_signal_down[i - 1]) != np.sign(sample_signal_down[i]):
                zeros.append(i)
            if np.sign(sample_signal_up[i - 1]) != np.sign(sample_signal_up[i]):
                zeros.append(i)

        ik_sol = circle_evaluations[zeros[3]].determine_joint_values(
            self.dh_params, O5, O6, self.fk, T_R0_tool
        )

        return (
            T_R0_tool,
            O6,
            O5,
            circle_evaluations,
            sample_signal_up,
            sample_signal_down,
            zeros,
            [np.degrees(J) for J in ik_sol],
        )


class DemoNode(Node):
    def __init__(self):
        super().__init__("demo_node")
        self.robot = CRXRobot()

        self.tf_broadcaster = ReBroadcastableStaticTransformBroadcaster(self)
        self.marker_publisher = self.create_publisher(MarkerArray, "markers", 10)
        self.plot_image_publisher = self.create_publisher(Image, "plot_img", 10)

        self.subscription = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        t = ((self.get_clock().now().nanoseconds / 1e9) % 10) / 10
        self.get_logger().info(f" {t=}")

        # angle = 22.5 * np.sin(2 * np.pi * t)
        # joint_values = [0, round(float(angle), 3), 0, 0, 0, 0]

        joint_values = [+78, -41, +17, -42, -60, +10]  # Eq 10
        # joint_values = [0] * 6
        self.get_logger().info(f" {joint_values=}")

        ### FK ###

        T_R0_tool, T_list = self.robot.fk(
            joint_values=joint_values, return_individual_transforms=True
        )
        xyzwpr = to_xyzwpr(T_R0_tool)
        self.get_logger().info(f" FK={[round(x, 3) for x  in xyzwpr]}")

        transforms = [
            TransformStamped(
                header=Header(frame_id=self.robot.frame_names[i]),
                child_frame_id=self.robot.frame_names[i + 1],
                transform=matrix_to_transform(T),
            )
            for i, T in enumerate(T_list)
        ]

        ### IK ###

        (
            T_R0_tool,
            O6,
            O5,
            circle_evaluations,
            sample_signal_up,
            sample_signal_down,
            zeros,
            ik_sol,
        ) = self.robot.ik(
            xyzwpr
        )  # [80.321, 287.676, 394.356, -131.819, -45.268, 61.453])

        i = 360 * (time.time() % 4) / 4
        ce = circle_evaluations[int(i)]

        fig, ax = plt.subplots()
        x = np.linspace(0, 2 * np.pi, 360)
        ax.plot(x, sample_signal_up, color="r", label="Z5Z4UP")
        ax.plot(x, sample_signal_down, color="b", label="Z5Z4DOWN")
        ax.axhline(y=0.0, color="black", linestyle="-")
        ax.axvline(x=np.radians(i), color="black", linestyle="-")
        for z in zeros:
            ax.axvline(x=circle_evaluations[z].q, color="black", linestyle="--")
        ax.set_title("Figure 7: Sample signal dot products")
        ax.legend()
        fig.canvas.draw()

        io_buf = io.BytesIO()
        fig.savefig(io_buf, format="raw")
        io_buf.seek(0)
        image_array = np.reshape(
            np.frombuffer(io_buf.getvalue(), dtype=np.uint8),
            newshape=(int(fig.bbox.bounds[3]), int(fig.bbox.bounds[2]), -1),
        )
        io_buf.close()
        plt.close(fig)

        self.plot_image_publisher.publish(CvBridge().cv2_to_imgmsg(image_array))

        T_R0_toolsol, T_listsol = self.robot.fk(ik_sol, return_individual_transforms=True)

        transforms.extend(
            [
                TransformStamped(
                    header=Header(frame_id="R0" if i == 0 else "sol_" + self.robot.frame_names[i]),
                    child_frame_id="sol_" + self.robot.frame_names[i + 1],
                    transform=matrix_to_transform(T),
                )
                for i, T in enumerate(T_listsol)
            ]
        )
        transforms.append(
            TransformStamped(
                header=Header(frame_id="R0"),
                child_frame_id="sol_tool",
                transform=matrix_to_transform(T_R0_toolsol),
            )
        )
        transforms.extend(
            [
                TransformStamped(
                    header=Header(frame_id="R0"),
                    child_frame_id="desired_pose",
                    transform=matrix_to_transform(T_R0_tool),
                ),
                TransformStamped(
                    header=Header(frame_id="R0"),
                    child_frame_id="plane",
                    transform=matrix_to_transform(ce.T_R0_plane),
                ),
            ]
        )

        markers = [
            Marker(
                header=Header(frame_id="R0"),
                ns="O_points",
                id=i,
                type=Marker.SPHERE,
                pose=Pose(position=Point(x=x, y=y, z=z)),
                scale=Vector3(x=0.02, y=0.02, z=0.02),
                color=ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.5),
            )
            for i, (x, y, z) in enumerate([O6, O5, [0.0, 0.0, 0.0]])
        ]
        markers.extend(ce.markers())
        markers.extend(
            [
                Marker(
                    header=Header(frame_id="R0"),
                    ns="O4_candidate_line_strip",
                    type=Marker.LINE_STRIP,
                    scale=Vector3(x=0.001, y=0.001, z=0.001),
                    color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5),
                    points=[numpy_to_point(ce.O4) for ce in circle_evaluations],
                ),
                Marker(
                    header=Header(frame_id="R0"),
                    ns="O3UP_line_strip",
                    type=Marker.LINE_STRIP,
                    scale=Vector3(x=0.001, y=0.001, z=0.001),
                    color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5),
                    points=[numpy_to_point(ce.O3UP) for ce in circle_evaluations],
                ),
                Marker(
                    header=Header(frame_id="R0"),
                    ns="O3DOWN_line_strip",
                    type=Marker.LINE_STRIP,
                    scale=Vector3(x=0.001, y=0.001, z=0.001),
                    color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5),
                    points=[numpy_to_point(ce.O3DOWN) for ce in circle_evaluations],
                ),
            ]
        )
        for i, m in enumerate(markers):
            m.id = i

        self.tf_broadcaster.sendTransform(transforms)
        self.marker_publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    # try:
    #     with rclpy.init(args=args):
    #         node = DemoNode()
    #         rclpy.spin(node)
    # except (KeyboardInterrupt, ExternalShutdownException):
    #     pass


if __name__ == "__main__":
    main()
