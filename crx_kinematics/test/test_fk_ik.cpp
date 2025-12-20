#include <gtest/gtest.h>

#include "crx_kinematics/robot.hpp"

TEST(TestCrxKinematics, test_xyzwpr_round_trip)
{
    for (auto i = 1u; i < 100; ++i)
    {
        Eigen::Vector3d v = Eigen::Vector3d::Random();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = v;
        T.linear() = Eigen::Matrix3d(Eigen::AngleAxis(v.norm(), v.normalized()));

        const std::array<double, 6> xyzwpr = crx_kinematics::to_xyzwpr(T);
        const Eigen::Isometry3d T_again = crx_kinematics::from_xyzwpr(xyzwpr);

        ASSERT_TRUE(T.translation().isApprox(T_again.translation()));
        ASSERT_TRUE(T.linear().isApprox(T_again.linear()));
    }
}

TEST(TestCrxKinematics, test_fk)
{
    const auto robot = crx_kinematics::CRXRobot();

    // Eq. 10
    std::array<double, 6> joint_values = crx_kinematics::deg2rad({ +78, -41, +17, -42, -60, +10 });
    const Eigen::Isometry3d T_R0_tool = robot.fk(joint_values);
    const std::array<double, 6> xyzwpr = crx_kinematics::to_xyzwpr(T_R0_tool);

    // Eq. 12 (CRX-10ia)
    const std::array<double, 6> expected = { 80.321, 287.676, 394.356, -131.819, -45.268, 61.453 };
    for (std::size_t i = 0u; i < 6; ++i)
    {
        ASSERT_NEAR(xyzwpr[i], expected[i], 1e-3);
    }
}

void assert_ik_solutions(const Eigen::Isometry3d& T_R0_tool,
                         std::vector<std::array<double, 6>>& expected)
{
    const auto robot = crx_kinematics::CRXRobot();
    std::vector<std::array<double, 6>> solutions = robot.ik(T_R0_tool);

    ASSERT_EQ(solutions.size(), expected.size());

    // Some solutions and expected solutions have J1, J4 or J5 at +-180 degrees.
    // Such solutions are identical, although leads to a large test diff.
    // Therefore, harmonize all J1 and J4 values to positive 180 if applicable.
    for (auto& sol : expected)
    {
        for (int i : { 0, 3, 4 })
        {
            sol[i] = std::abs(180.0 + sol[i]) < 1e-3 ? sol[i] + 360.0 : sol[i];
        }
    }
    for (auto& sol : solutions)
    {
        for (int i : { 0, 3, 4 })
        {
            sol[i] = std::abs(M_PI + sol[i]) < 1e-3 ? sol[i] + (2.0 * M_PI) : sol[i];
        }
    }

    std::sort(solutions.begin(), solutions.end(), [](const auto& a, const auto& b) {
        return a[0] != b[0] ? a[0] < b[0] : a[1] < b[1];
    });
    std::sort(expected.begin(), expected.end(), [](const auto& a, const auto& b) {
        return a[0] != b[0] ? a[0] < b[0] : a[1] < b[1];
    });

    for (auto i = 0u; i < solutions.size(); ++i)
    {
        for (auto j = 0u; j < 6; ++j)
        {
            ASSERT_NEAR(solutions[i][j] / M_PI * 180.0, expected[i][j], 1e-2) << i << "," << j;
        }
    }

    // Check that FK on each solution brings us back to where we started
    for (const auto& solution : solutions)
    {
        const Eigen::Isometry3d T_again = robot.fk(solution);
        ASSERT_TRUE(T_R0_tool.translation().isApprox(T_again.translation(), 1e-6))
            << T_R0_tool.translation().transpose() << "\nvs " << T_again.translation().transpose();
        ASSERT_TRUE(T_R0_tool.linear().isApprox(T_again.linear(), 1e-6))
            << T_R0_tool.linear() << "\nvs\n"
            << T_again.linear();
    }
}

TEST(TestCrxKinematics, test_ik_example_3_1)
{
    const Eigen::Isometry3d T_R0_tool =
        crx_kinematics::from_xyzwpr({ 80.321, 287.676, 394.356, -131.819, -45.268, 61.453 });

    // Table 4
    std::vector<std::array<double, 6>> expected_solutions = {
        // 8 solutions
        { 44.611, 89.087, 109.193, 94.703, 121.416, 121.782 },
        { 35.162, 88.468, 140.150, -108.846, -111.920, -91.804 },
        { 29.462, -39.473, -8.392, 117.682, 85.679, -119.224 },
        { 78, -41, 17, -42, -60, 10 },
        { -135.389, -89.087, 70.807, -85.297, 121.416, 121.782 },
        { -144.839, -88.468, 39.850, 71.154, -111.920, -91.804 },
        { -150.538, 39.473, 188.392 - 360, -62.318, 85.679, -119.224 },
        { -102, 41, 163, 138, -60, 10 },
    };
    // NOTE: "188.392" is the only occurrence in the paper of a value not harmonized towards
    // zero [-180, 180].
    // We harmonize it to make it consistent, and to make the test pass !

    assert_ik_solutions(T_R0_tool, expected_solutions);
}

TEST(TestCrxKinematics, test_ik_example_3_2)
{
    const Eigen::Isometry3d T_R0_tool = crx_kinematics::from_xyzwpr({ 600, 0, 100, 180, 0, 70 });

    // Table 5
    std::vector<std::array<double, 6>> expected_solutions = {
        // 12 solutions
        { -14.478, 119.780, 78.001, -180, 168.001, -95.522 },
        { -6.336, 121.233, 89.999, -116.197, 179.999, -39.861 },
        { 6.335, 121.233, 90, -63.811, 179.999, -0.146 },
        { 14.478, 119.780, 78.001, 0, -168.001, 55.522 },
        { -14.478, 11.999, -29.780, -180, 60.220, -95.522 },
        { 14.478, 11.999, -29.780, 0, -60.220, 55.522 },
        { 165.522, -119.780, 101.999, 0, 168.001, -95.522 },
        { 173.664, -121.233, 90.001, 63.811, 179.999, -39.861 },
        { -173.665, -121.233, 90, 116.190, 179.999, -0.146 },
        { -165.522, -119.780, 101.999, -180, -168.01, 55.522 },
        { 165.522, -11.999, -150.220, 0, 60.220, -95.522 },
        { -165.522, -11.999, -150.220, 180, -60.220, 55.522 },
    };

    assert_ik_solutions(T_R0_tool, expected_solutions);
}

TEST(TestCrxKinematics, test_ik_example_3_3)
{
    const Eigen::Isometry3d T_R0_tool =
        crx_kinematics::from_xyzwpr({ 209.470, -42.894, 685.496, -95.378, -64.226, -56.402 });

    // Table 6
    std::vector<std::array<double, 6>> expected_solutions = {
        // 16 solutions
        { -60.125, 62.707, 112.015, 90.165, 92.586, 132.291 },
        { -63.318, 62.684, 143.064, -93.111, -89.750, -78.691 },
        { 11.855, 54.151, 144.007, -28.773, -142.889, -48.616 },
        { 49.247, 46.825, 135.954, 29.379, -134.512, -5.010 },
        { -62.156, -40.094, 14.333, 92.039, 91.458, -129.964 },
        { 47.115, -53.924, 35.922, 28.105, -41.924, -48.121 },
        { 0, -45, 44, -37, -53, 0 },
        { -47.369, -40.310, 45.272, -78.410, -81.969, 17.952 },
        { 119.875, -62.707, 67.985, -89.835, 92.586, 132.291 },
        { 116.682, -62.684, 36.936, 86.889, -89.750, -78.691 },
        { -168.145, -54.151, 35.993, 151.227, -142.889, -48.616 },
        { -130.753, -46.825, 44.046, -150.621, -134.512, -5.01 },
        { 117.844, 40.094, 165.667, -87.961, 91.458, -129.964 },
        { -132.885, 53.924, 144.078, -151.895, -41.924, -48.121 },
        { -180, 45, 136, 143, -53, 0 },
        { 132.631, 40.310, 134.728, 101.590, -81.969, 17.952 },
    };

    assert_ik_solutions(T_R0_tool, expected_solutions);
}
