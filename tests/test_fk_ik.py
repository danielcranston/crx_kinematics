import numpy as np

from crx_kinematics.demo_node import from_xyzwpr, to_xyzwpr
from crx_kinematics.robot import CRXRobot

assert_allclose = lambda a, b, atol=1e-3, err_msg="": np.testing.assert_allclose(
    a, b, atol=atol, err_msg=err_msg
)


def test_fk():
    robot = CRXRobot()

    # Eq 10
    joint_values = [+78, -41, +17, -42, -60, +10]

    T_R0_tool = robot.fk(joint_values=joint_values)
    xyzwpr = to_xyzwpr(T_R0_tool)

    # Eq 12 (CRX-10iA)
    assert_allclose(xyzwpr, [80.321, 287.676, 394.356, -131.819, -45.268, 61.453])


def assert_ik_solutions(xyzwpr, expected_solutions):
    robot = CRXRobot()

    T_R0_tool = from_xyzwpr(xyzwpr)

    (
        circle_evaluations,
        sample_signal_up,
        sample_signal_down,
        up_zeros,
        down_zeros,
        ik_sols,
    ) = robot.ik(T_R0_tool)

    assert len(ik_sols) == len(expected_solutions) / 2  # TODO: Implement Step 7, use full length

    associated_expected_idxs = [
        np.argmin(np.sum(np.abs(np.subtract(expected_solutions, sol)[:, :3]), axis=1))
        for sol in ik_sols
    ]

    assert len(associated_expected_idxs) == len(set(associated_expected_idxs))

    def harmonized_diff(solution, expected):
        "Some angles in the Tables of expected values (or WPRs) have +180 where -180 produces the same result"
        diff = np.abs(np.subtract(solution, expected))
        return np.array([x - 360 if np.isclose(x, 360, atol=0.1) else x for x in diff])

    for solution, expected_idx in zip(ik_sols, associated_expected_idxs):
        diff = harmonized_diff(solution, expected_solutions[expected_idx])
        assert_allclose(
            diff,
            np.zeros(6),
            atol=1e-2,
            err_msg=f"\n{ik_sols} vs\n{expected_solutions[expected_idx]}. \ndiff={diff}",
        )

    # Check that FK on each solution brings us back to where we started
    T_R0_tool_list = [robot.fk(solution) for solution in ik_sols]
    for T in T_R0_tool_list:
        assert_allclose(T, T_R0_tool)

    for xyzwpr_sol in [to_xyzwpr(T) for T in T_R0_tool_list]:
        diff = harmonized_diff(xyzwpr_sol, xyzwpr)
        assert_allclose(diff, np.zeros(6), err_msg=f"\n{xyzwpr_sol} vs\n{xyzwpr}. \ndiff={diff}")


def test_ik_example_3_1():
    xyzwpr = [80.321, 287.676, 394.356, -131.819, -45.268, 61.453]

    # Table 4
    expected_solutions = [  # 8 solutions
        [44.611, 89.087, 109.193, 94.703, 121.416, 121.782],
        [35.162, 88.468, 140.150, -108.846, -111.920, -91.804],
        [29.462, -39.473, -8.392, 117.682, 85.679, -119.224],
        [78, -41, 17, -42, -60, 10],
        [-135.389, -89.087, 70.807, -85.297, 121.416, 121.782],
        [-144.839, -88.468, 39.850, 71.154, -111.920, -91.804],
        [-150.538, 39.473, 188.392, -62.318, 85.679, -119.224],
        [-102, 41, 163, 138, -60, 10],
    ]

    assert_ik_solutions(xyzwpr, expected_solutions)


def test_ik_example_3_2():
    xyzwpr = [600, 0, 100, -180, 0, 70]
    robot = CRXRobot()

    # Table 5
    expected_solutions = [  # 12 solutions
        # Note: +180 OR -180 on J4 leads to same pose! Hence we need the "harmonized_diff" fn above
        [-14.478, 119.780, 78.001, -180, 168.001, -95.522],
        [-6.336, 121.233, 89.999, -116.197, 179.999, -39.861],
        [6.335, 121.233, 90, -63.811, 179.999, -0.146],
        [14.478, 119.780, 78.001, 0, -168.001, 55.522],
        [-14.478, 11.999, -29.780, -180, 60.220, -95.522],
        [14.478, 11.999, -29.780, 0, -60.220, 55.522],
        [165.522, -119.780, 101.999, 0, 168.001, -95.522],
        [173.664, -121.233, 90.001, 63.811, 179.999, -39.861],
        [-173.665, -121.233, 90, 116.190, 179.999, -0.146],
        [-165.522, -119.780, 101.999, -180, -168.01, 55.522],
        [165.522, -11.999, -150.220, 0, 60.220, -95.522],
        [-165.522, -11.999, -150.220, 180, -60.220, 55.522],
    ]

    assert_ik_solutions(xyzwpr, expected_solutions)


def test_example_3_3():
    xyzwpr = [209.470, -42.894, 685.496, -95.378, -64.226, -56.402]

    # Table 6
    expected_solutions = [
        [-60.125, 62.707, 112.015, 90.165, 92.586, 132.291],
        [-63.318, 62.684, 143.064, -93.111, -89.750, -78.691],
        [11.855, 54.151, 144.007, -28.773, -142.889, -48.616],
        [49.247, 46.825, 135.954, 29.379, -134.512, -5.010],
        [-62.156, -40.094, 14.333, 92.039, 91.458, -129.964],
        [47.115, -53.924, 35.922, 28.105, -41.924, -48.121],
        [0, -45, 44, -37, -53, 0],
        [-47.369, -40.310, 45.272, -78.410, -81.969, 17.952],
        [119.875, -62.707, 67.985, -89.835, 92.586, 132.291],
        [116.682, -62.684, 36.936, 86.889, -89.750, -78.691],
        [-168.145, -54.151, 35.993, 151.227, -142.889, -48.616],
        [-130.753, -46.825, 44.046, -150.621, -134.512, -5.01],
        [117.844, 40.094, 165.667, -87.961, 91.458, -129.964],
        [-132.885, 53.924, 144.078, -151.895, -41.924, -48.121],
        [-180, 45, 136, 143, -53, 0],
        [132.631, 40.310, 134.728, 101.590, -81.969, 17.952],
    ]

    assert_ik_solutions(xyzwpr, expected_solutions)
