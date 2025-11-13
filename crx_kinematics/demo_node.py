import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf_transformations as tr

from sensor_msgs.msg import Image
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import MarkerArray

from crx_kinematics.robot import CRXRobot
from crx_kinematics.utils.geometry import get_dual_ik_solution, harmonize_towards_zero
from crx_kinematics.utils.visualization import (
    add_robot_joint_markers,
    create_marker_array,
    create_transforms,
    make_plot_img,
)

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
        self.sol_idx = 0
        self.create_service(Empty, "next_sol", self.next_sol_cb)
        self.joint_value_idx = -2
        self.joint_values = [
            [+78, -41, +17, -42, -60, +10],  # Eq 10
            [-14.478, 11.999, -29.780, -180, 60.220, -95.522],  # JC#5 in Table 5
            [0] * 6,
            [78, -41, 17, -42, -60, 10],  # JB#4 in Table 4
            [-150.538, 39.473, 188.392, -62.318, 85.679, -119.224],  #  JB#7 in Table 4
            [-150.537, 39.472, -171.608, -62.318, 85.679, -119.224],  # Our IK
        ]
        self.create_service(Trigger, "next_joint_value", self.next_joint_value_cb)
        self.use_dual_joint_pose = False
        self.create_service(Empty, "toggle_dual_joint_pose", self.toggle_dual_joint_pose_cb)

    def toggle_dual_joint_pose_cb(self, _, response):
        self.use_dual_joint_pose = not self.use_dual_joint_pose
        return response

    def next_sol_cb(self, _, response):
        self.sol_idx = self.sol_idx + 1
        return response

    def next_joint_value_cb(self, _, response):
        self.joint_value_idx = (self.joint_value_idx + 1) % len(self.joint_values)
        response.success = True
        response.message = f"Using joint value {self.joint_value_idx}"
        return response

    def pause_resume_cb(self, _, response):
        self.paused = not self.paused
        return response

    def timer_cb(self):
        # t = ((self.get_clock().now().nanoseconds / 1e9) % 10) / 10
        # angle = 22.5 * np.sin(2 * np.pi * t)
        # joint_values = [0, round(float(angle), 3), 0, 0, 0, 0]
        joint_values = self.joint_values[self.joint_value_idx]
        if self.use_dual_joint_pose:
            joint_values = harmonize_towards_zero(get_dual_ik_solution(joint_values))
        self.get_logger().info(f" {joint_values=}")

        ### FK ###

        T_R0_tool, T_list = self.robot.fk(
            joint_values=joint_values, return_individual_transforms=True
        )
        self.get_logger().info(f" FK={[round(x, 3) for x in to_xyzwpr(T_R0_tool)]}")

        ### IK ###

        ik_sols, debug_data = self.robot.ik(T_R0_tool)

        i = 360 * (0 if self.paused else time.time() % 10) / 10
        ce = debug_data.circle_evaluations[int(i)]

        zeros = debug_data.up_zeros + debug_data.down_zeros
        zeros_distances_from_current = [abs(q - ce.q) for q in zeros]
        ik_sol = ik_sols[np.argmin(zeros_distances_from_current)]
        # ik_sol = ik_sols[self.sol_idx % len(zeros)]
        self.get_logger().info(f" IK={[float(round(x, 3)) for x in ik_sol]}")

        _, T_listsol = self.robot.fk(ik_sol, return_individual_transforms=True)

        ### Visualizations ###

        marker_array = create_marker_array(ce, debug_data.circle_evaluations)
        add_robot_joint_markers(marker_array)
        self.marker_publisher.publish(marker_array)

        now = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(
            create_transforms(
                T_list, T_listsol, T_R0_tool, ce.T_R0_plane, self.robot.frame_names, now
            )
        )

        image_array = make_plot_img(debug_data, i)
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
