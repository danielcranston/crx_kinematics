from dataclasses import dataclass

import numpy as np

from crx_kinematics.utils.geometry import (
    construct_plane,
    find_third_triangle_corner,
    find_zeros,
    isometry_inv,
    normalized,
)


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

    def determine_joint_values(self, dh_params, O5, O6, T_R0_tool, is_up):
        J1 = np.arctan2(self.O4[1], self.O4[0])

        T_L1_L0 = isometry_inv(dh_params[0].T(J1))
        R_L1_L0 = T_L1_L0[:3, :3]
        O_1_3 = R_L1_L0 @ (self.O3UP if is_up else self.O3DOWN)
        J2 = -np.arctan2(O_1_3[2], O_1_3[0]) + np.pi / 2

        O_1_4 = R_L1_L0 @ self.O4
        J3 = np.arctan2(O_1_4[2] - O_1_3[2], O_1_4[0] - O_1_3[0])

        T_L2_L1 = isometry_inv(dh_params[1].T(J2))
        T_L3_L2 = isometry_inv(dh_params[2].T(J3 + J2))  # Account for "Fanuc pecularity"
        T_L3_L0 = T_L3_L2 @ T_L2_L1 @ T_L1_L0
        O_3_5 = T_L3_L0 @ [*O5, 1.0]
        J4 = np.arctan2(O_3_5[0], O_3_5[2])

        T_L4_L3 = isometry_inv(dh_params[3].T(J4))
        T_L4_L0 = T_L4_L3 @ T_L3_L0
        O_4_6 = T_L4_L0 @ [*O6, 1.0]
        J5 = np.arctan2(O_4_6[0], -O_4_6[2])

        T_L5_L0 = isometry_inv(dh_params[4].T(J5)) @ T_L4_L0
        T_L5_tool = T_L5_L0 @ T_R0_tool
        J6 = np.arctan2(-T_L5_tool[2, 0], T_L5_tool[0, 0])

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
        sample_signal_up, sample_signal_down, sample_signal_x = zip(
            *[(ce.dot_product_up, ce.dot_product_down, ce.q) for ce in circle_evaluations]
        )

        # Step 5: Find the Ns zeros of the sample functions
        def sample_fn_eval(q, is_up):
            ce = CircleEvaluation(q, T_R0_tool, self.dh_params, O5)
            return ce.dot_product_up if is_up else ce.dot_product_down

        up_zeros, down_zeros = find_zeros(
            sample_signal_up, sample_signal_down, sample_signal_x, fn=sample_fn_eval
        )

        # Step 6: Position O4 and O3, determine joint values geometrically
        ik_sols = [
            CircleEvaluation(root, T_R0_tool, self.dh_params, O5).determine_joint_values(
                self.dh_params, O5, O6, T_R0_tool, is_up
            )
            for root, is_up in zip(
                up_zeros + down_zeros, [True] * len(up_zeros) + [False] * len(down_zeros)
            )
        ]

        # Step 7: Calculate dual solutions for each of the above IK solutions

        return (
            circle_evaluations,
            sample_signal_up,
            sample_signal_down,
            up_zeros,
            down_zeros,
            ik_sols,
        )
