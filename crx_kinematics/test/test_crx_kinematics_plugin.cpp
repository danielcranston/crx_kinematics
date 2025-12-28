#include <algorithm>
#include <filesystem>
#include <random>

#include <gtest/gtest.h>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <urdf_parser/urdf_parser.h>

#include "crx_kinematics/crx_kinematics_plugin.hpp"

std::vector<double> generate_random_joint_values()
{
    auto random_joint_value = []() {
        static std::uniform_int_distribution<int> distr{ -89, 89 };
        static std::mt19937 noise{ 0 };
        return distr(noise) / 180.0 * M_PI;
    };

    std::vector<double> out;
    std::generate_n(std::back_inserter(out), 6, random_joint_value);
    return out;
}

geometry_msgs::msg::Pose do_fk(const crx_kinematics::CRXKinematicsPlugin& plugin,
                               const std::vector<double>& joint_values)
{
    std::vector<geometry_msgs::msg::Pose> fk_poses;
    plugin.getPositionFK({ "flange" }, joint_values, fk_poses);
    return fk_poses[0];
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> pose_to_eigen(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Isometry3d T;
    tf2::fromMsg(pose, T);
    return std::make_pair(T.translation(), Eigen::Quaterniond(T.linear()));
}

void print(const auto& v)
{
    for (auto i : v)
        std::cout << i << ' ';
    std::cout << "\n";
}

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

void assert_no_ik_for_unreachable_pose(const crx_kinematics::CRXKinematicsPlugin& plugin)
{
    geometry_msgs::msg::Pose unreachable_pose;
    unreachable_pose.position.x = 100.0;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    std::vector<double> solution;
    plugin.getPositionIK(unreachable_pose, {}, solution, error_code);
    ASSERT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

void assert_fk_ik_round_trip(const crx_kinematics::CRXKinematicsPlugin& plugin,
                             const std::vector<double>& joint_values)
{
    const geometry_msgs::msg::Pose fk_pose = do_fk(plugin, joint_values);

    moveit_msgs::msg::MoveItErrorCodes error_code;
    std::vector<double> solution;
    plugin.getPositionIK(fk_pose, joint_values, solution, error_code);
    ASSERT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

    const geometry_msgs::msg::Pose fk_pose_again = do_fk(plugin, solution);

    const auto [p1, q1] = pose_to_eigen(fk_pose);
    const auto [p2, q2] = pose_to_eigen(fk_pose_again);
    ASSERT_NEAR((p1 - p2).norm(), 0.0, 1e-6);
    ASSERT_NEAR(q1.angularDistance(q2), 0.0, 1e-6);
}

void assert_all_zeros_pose(const crx_kinematics::CRXKinematicsPlugin& plugin,
                           const Eigen::Vector3d& expected_position)
{
    const std::vector<double> all_zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    const geometry_msgs::msg::Pose fk_pose = do_fk(plugin, all_zeros);
    const auto [position, orientation] = pose_to_eigen(fk_pose);

    const auto expected_orientation = Eigen::Quaterniond(/*w=*/1.0, 0.0, 0.0, 0.0);

    ASSERT_NEAR((position - expected_position).norm(), 0.0, 1e-6);
    ASSERT_NEAR(orientation.angularDistance(expected_orientation), 0.0, 1e-6);

    assert_fk_ik_round_trip(plugin, all_zeros);
}

TEST(CrxKinematicsPluginTest, test_plugin_crx10ia)
{
    const auto plugin = make_plugin("crx10ia");
    assert_no_ik_for_unreachable_pose(plugin);
    assert_all_zeros_pose(plugin, Eigen::Vector3d(0.7, -0.15, 0.245 + 0.54));
    for (int i = 0; i < 1000; ++i)
    {
        const auto joint_values = generate_random_joint_values();
        // print(joint_values);
        assert_fk_ik_round_trip(plugin, joint_values);
    }
}

TEST(CrxKinematicsPluginTest, test_plugin_crx30ia)
{
    const auto plugin = make_plugin("crx30ia");
    assert_no_ik_for_unreachable_pose(plugin);
    assert_all_zeros_pose(plugin, Eigen::Vector3d(0.93, -0.185, 0.37 + 0.95));
    for (int i = 0; i < 1000; ++i)
    {
        const auto joint_values = generate_random_joint_values();
        // print(joint_values);
        assert_fk_ik_round_trip(plugin, joint_values);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(0, nullptr);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
