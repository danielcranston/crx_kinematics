from dataclasses import dataclass
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.logging
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf_transformations as tr

from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray

from crx_kinematics.utils.geometry import *
from crx_kinematics.utils.visualization import create_marker_array, create_transforms, make_plot_img

"""
delta: offset along previous Z to the common normal
theta: angle about previous z from old x to new x
r (or a): length of the common normal. The radius about previous z
alpha: angle about common normal, from old z axis to new z axis
"""


def to_xyzwpr(T):
    xyz = (1000 * T[:3, 3]).tolist()
    wpr = np.array([np.degrees(a) for a in tr.euler_from_matrix(T)]).tolist()

    return xyz + wpr


def from_xyzwpr(xyzwpr):
    T = tr.euler_matrix(*[np.radians(a) for a in xyzwpr[3:]])
    T[:3, 3] = [p / 1000 for p in xyzwpr[:3]]

    return T


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

        # Find O3 in that plane.
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

    def determine_joint_values(self, dh_params, O5, O6, fk, T_R0_tool, is_up):
        J1 = np.atan2(self.O4[1], self.O4[0])

        R_L1_L0 = dh_params[0].T(J1)[:3, :3].T
        O_1_3 = R_L1_L0 @ self.O3UP
        J2 = -np.atan2(O_1_3[2], O_1_3[0]) + np.pi / 2

        O_1_4 = R_L1_L0 @ self.O4
        J3 = np.atan2(O_1_4[2] - O_1_3[2], O_1_4[0] - O_1_3[0])

        O_1_5 = R_L1_L0 @ O5
        O4O5_dir = O_1_5 - O_1_4
        J4 = np.atan2(O4O5_dir[2], -O4O5_dir[1])

        T_R0_L4 = fk([np.degrees(j) for j in [J1, J2, J3, J4, 0, 0]], up_to=4)

        O_4_6 = isometry_inv(T_R0_L4) @ [*O6, 1.0]
        J5 = np.atan2(O_4_6[0], -O_4_6[2])

        T_R0_L5 = T_R0_L4 @ dh_params[4].T(J5)
        T_L5_tool = isometry_inv(T_R0_L5) @ T_R0_tool

        J6 = np.atan2(-T_L5_tool[2, 0], T_L5_tool[0, 0])

        solution = [np.degrees(J) for J in [J1, J2, J3, J4, J5, J6]]
        return solution


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

    def ik(self, T_R0_tool):
        # Step 1: Positioning the centers O6 and O5 in the frame R0
        O6 = T_R0_tool[:3, 3]
        O5 = (T_R0_tool @ [0, 0, self.L6.r, 1])[:3]

        # Step 2: Position all O4 candidates on a circle. Each candidate is a CircleEvaluation
        circle_evaluations = [
            CircleEvaluation(q, T_R0_tool, self.dh_params, O5)
            for q in np.linspace(0, 2 * np.pi, 360)
        ]
        # Each CircleEvaluation also performs
        #  - Step 3: Position the 2 O3 candidates (O3UP, O3DOWN) in the vertical O0O3O4 plane
        #  - Calculate the dot products Z4Z5 for UP and DOWN

        # Step 4: Calculate the sample functions (dot_product_up, dot_product_down)
        sample_signal_up, sample_signal_down = zip(
            *[(ce.dot_product_up, ce.dot_product_down) for ce in circle_evaluations]
        )

        # Step 5: Find the Ns zeros of the sample functions
        up_zeros = []
        down_zeros = []
        for i in range(1, len(sample_signal_down)):
            if np.sign(sample_signal_down[i - 1]) != np.sign(sample_signal_down[i]):
                down_zeros.append(i)
            if np.sign(sample_signal_up[i - 1]) != np.sign(sample_signal_up[i]):
                up_zeros.append(i)

        # Step 6
        ik_sols = [
            circle_evaluations[zero_idx].determine_joint_values(
                self.dh_params, O5, O6, self.fk, T_R0_tool, is_up
            )
            for zero_idx, is_up in zip(
                up_zeros + down_zeros, [True] * len(up_zeros) + [False] * len(down_zeros)
            )
        ]

        return (
            circle_evaluations,
            sample_signal_up,
            sample_signal_down,
            up_zeros,
            down_zeros,
            ik_sols,
        )


class DemoNode(Node):
    def __init__(self):
        super().__init__("demo_node")
        self.robot = CRXRobot()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_publisher = self.create_publisher(MarkerArray, "markers", 10)
        self.plot_image_publisher = self.create_publisher(Image, "plot_img", 10)

        self.subscription = self.create_timer(0.1, self.timer_cb)
        self.paused = False
        self.create_service(Empty, "pause_resume", self.pause_resume_cb)

    def pause_resume_cb(self, _, response):
        self.paused = not self.paused
        return response

    def timer_cb(self):
        # t = ((self.get_clock().now().nanoseconds / 1e9) % 10) / 10
        # angle = 22.5 * np.sin(2 * np.pi * t)
        # joint_values = [0, round(float(angle), 3), 0, 0, 0, 0]

        joint_values = [+78, -41, +17, -42, -60, +10]  # Eq 10
        # joint_values = [0] * 6
        self.get_logger().info(f" {joint_values=}")

        ### FK ###

        T_R0_tool, T_list = self.robot.fk(
            joint_values=joint_values, return_individual_transforms=True
        )
        self.get_logger().info(f" FK={[round(x, 3) for x in to_xyzwpr(T_R0_tool)]}")

        ### IK ###

        (
            circle_evaluations,
            sample_signal_up,
            sample_signal_down,
            up_zeros,
            down_zeros,
            ik_sols,
        ) = self.robot.ik(T_R0_tool)
        ik_sol = ik_sols[1]
        self.get_logger().info(f" IK={[float(round(x, 3)) for x in ik_sol]}")

        i = 360 * (0 if self.paused else time.time() % 10) / 10
        ce = circle_evaluations[int(i)]

        _, T_listsol = self.robot.fk(ik_sol, return_individual_transforms=True)

        ### Visualizations ###

        self.marker_publisher.publish(create_marker_array(ce, circle_evaluations))
        now = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(
            create_transforms(
                T_list, T_listsol, T_R0_tool, ce.T_R0_plane, self.robot.frame_names, now
            )
        )
        image_array = make_plot_img(
            sample_signal_up, sample_signal_down, up_zeros + down_zeros, circle_evaluations, i
        )
        self.plot_image_publisher.publish(CvBridge().cv2_to_imgmsg(image_array))


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
