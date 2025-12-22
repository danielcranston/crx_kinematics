#include "crx_kinematics/crx_kinematics_plugin.hpp"

#include <filesystem>

#include <gtest/gtest.h>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>
#include <tf2_eigen/tf2_eigen.hpp>

std::shared_ptr<moveit::core::RobotModel> load_crx_robot_model(const std::string& robot_name)
{
    const auto urdf_path = std::filesystem::path(__FILE__).parent_path() / (robot_name + ".urdf");
    const auto srdf_path = std::filesystem::path(__FILE__).parent_path() / (robot_name + ".srdf");

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_path.string());
    if (urdf_model == nullptr)
    {
        throw std::runtime_error("Could not load URDF");
    }

    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initFile(*urdf_model, srdf_path.string()))
    {
        throw std::runtime_error("Could not load SRDF");
    }

    return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

crx_kinematics::CRXKinematicsPlugin make_plugin(const std::string& robot_name)
{
    auto node = rclcpp::Node::make_shared("test_crx_kinematics_plugin");
    auto robot_model = load_crx_robot_model(robot_name);

    auto plugin = crx_kinematics::CRXKinematicsPlugin();
    if (!plugin.initialize(node, *robot_model, "manipulator", "base_link", { "flange" }, 0.0))
    {
        throw std::runtime_error("Could not initialize plugin for " + robot_name);
    }

    return plugin;
}

void assert_fk_ik_round_trip(const crx_kinematics::CRXKinematicsPlugin& plugin,
                             const geometry_msgs::msg::Point expected_fk_position)
{
    std::vector<double> joint_values = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // Compute FK, confirm that we get expected values
    std::vector<geometry_msgs::msg::Pose> fk_poses;
    plugin.getPositionFK({ "flange" }, joint_values, fk_poses);

    ASSERT_EQ(fk_poses.size(), 1);
    ASSERT_NEAR(fk_poses[0].position.x, expected_fk_position.x, 1e-6);
    ASSERT_NEAR(fk_poses[0].position.y, expected_fk_position.y, 1e-6);
    ASSERT_NEAR(fk_poses[0].position.z, expected_fk_position.z, 1e-6);

    // Note: Official URDFs have base frame orientation identical to tip frame
    const auto expected_fk_quat = Eigen::Quaterniond(/*w=*/1, 0, 0, 0);
    Eigen::Quaterniond quat;
    tf2::fromMsg(fk_poses[0].orientation, quat);
    ASSERT_NEAR(quat.angularDistance(expected_fk_quat), 0.0, 1e-6);

    // Compute IK, confirm that it succeeds to find a solution
    moveit_msgs::msg::MoveItErrorCodes error_code;
    std::vector<double> solution;
    plugin.getPositionIK(fk_poses[0], {}, solution, error_code);
    EXPECT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

    // Feed the solution back into FK and see that we landed back where we started
    std::vector<geometry_msgs::msg::Pose> fk_poses_again;
    plugin.getPositionFK({ "flange" }, joint_values, fk_poses_again);
    ASSERT_EQ(fk_poses_again.size(), 1);
    ASSERT_NEAR(fk_poses_again[0].position.x, fk_poses[0].position.x, 1e-6);
    ASSERT_NEAR(fk_poses_again[0].position.y, fk_poses[0].position.y, 1e-6);
    ASSERT_NEAR(fk_poses_again[0].position.z, fk_poses[0].position.z, 1e-6);
    ASSERT_NEAR(fk_poses_again[0].orientation.x, fk_poses[0].orientation.x, 1e-6);
    ASSERT_NEAR(fk_poses_again[0].orientation.y, fk_poses[0].orientation.y, 1e-6);
    ASSERT_NEAR(fk_poses_again[0].orientation.z, fk_poses[0].orientation.z, 1e-6);
    ASSERT_NEAR(fk_poses_again[0].orientation.w, fk_poses[0].orientation.w, 1e-6);
}

void assert_no_ik_for_unreachable_pose(const crx_kinematics::CRXKinematicsPlugin& plugin)
{
    geometry_msgs::msg::Pose unreachable_pose;
    unreachable_pose.position.x = 100.0;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    std::vector<double> solution;
    plugin.getPositionIK({ unreachable_pose }, {}, solution, error_code);
    EXPECT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

TEST(CrxKinematicsPluginTest, test_plugin_crx10ia)
{
    const auto plugin = make_plugin("crx10ia");
    geometry_msgs::msg::Point expected_fk_position;
    expected_fk_position.x = 0.7;
    expected_fk_position.y = -0.15;
    expected_fk_position.z = 0.245 + 0.54;  // base_to_R0 + R0_to_position_height
    assert_fk_ik_round_trip(plugin, expected_fk_position);
    assert_no_ik_for_unreachable_pose(plugin);
}

TEST(CrxKinematicsPluginTest, test_plugin_crx30ia)
{
    const auto plugin = make_plugin("crx30ia");
    geometry_msgs::msg::Point expected_fk_position;
    expected_fk_position.x = 0.93;
    expected_fk_position.y = -0.185;
    expected_fk_position.z = 0.37 + 0.95;  // base_to_R0 + R0_to_position_height
    assert_fk_ik_round_trip(plugin, expected_fk_position);
    assert_no_ik_for_unreachable_pose(plugin);
}

int main(int argc, char** argv)
{
    rclcpp::init(0, nullptr);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
