#include <algorithm>
#include <filesystem>
#include <random>
#include <sstream>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <urdf_parser/urdf_parser.h>

#if RCLCPP_VERSION_GTE(28, 1, 0)  // Jazzy or newer
#include <moveit/utils/robot_model_test_utils.hpp>
#else
#include <moveit/utils/robot_model_test_utils.h>
#endif

#include "crx_kinematics/crx_kinematics_plugin.hpp"

std::vector<double> generate_random_joint_values()
{
    auto random_joint_value = [](const int lower, const int upper) {
        std::uniform_int_distribution<int> distr{ lower, upper };
        static std::mt19937 noise{ 0 };
        return distr(noise) / 180.0 * M_PI;
    };

    return {
        random_joint_value(-179, 179),  //
        random_joint_value(-179, 179),  //
        random_joint_value(-69, 69),    /* Ensure we stay away from the edge of the robot envelope
                                         (fully extended or completely folder in towards the base),
                                         since randomized IK seems unreliable in these areas.
                                         This can (should be able to) be reverted once `find_zeros`
                                         is improved to do more than just applying the basic
                                         Bisection Method.*/
        random_joint_value(-179, 179),  //
        random_joint_value(-179, 179),  //
        random_joint_value(-179, 179)   //
    };
}

geometry_msgs::msg::Pose do_fk(const crx_kinematics::CRXKinematicsPlugin& plugin,
                               const std::vector<double>& joint_values)
{
    std::vector<geometry_msgs::msg::Pose> fk_poses;
    plugin.getPositionFK({ plugin.getTipFrame() }, joint_values, fk_poses);
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

std::string to_str(const auto& v)
{
    std::stringstream out;
    out << "[";
    for (std::size_t i = 0; i < v.size() - 1; ++i)
    {
        out << v[i] << ", ";
    }
    out << v[v.size() - 1] << "]";
    return out.str();
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

crx_kinematics::CRXKinematicsPlugin make_plugin(const std::string& robot_name,
                                                const std::string tip_frame = "flange")
{
    auto node = rclcpp::Node::make_shared("test_crx_kinematics_plugin");
    auto robot_model = load_crx_robot_model(robot_name);

    auto plugin = crx_kinematics::CRXKinematicsPlugin();
    if (!plugin.initialize(node, *robot_model, "manipulator", "base_link", { tip_frame }, 0.0))
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
    ASSERT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS) << to_str(joint_values);

    const geometry_msgs::msg::Pose fk_pose_again = do_fk(plugin, solution);

    const auto [p1, q1] = pose_to_eigen(fk_pose);
    const auto [p2, q2] = pose_to_eigen(fk_pose_again);
    ASSERT_NEAR((p1 - p2).norm(), 0.0, 1e-6) << to_str(joint_values);
    ASSERT_NEAR(q1.angularDistance(q2), 0.0, 1e-6) << to_str(joint_values);
}

void assert_all_zeros_pose(const crx_kinematics::CRXKinematicsPlugin& plugin,
                           const Eigen::Vector3d& expected_position,
                           const Eigen::Quaterniond& expected_orientation)
{
    const std::vector<double> all_zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    const geometry_msgs::msg::Pose fk_pose = do_fk(plugin, all_zeros);
    const auto [position, orientation] = pose_to_eigen(fk_pose);

    ASSERT_NEAR((position - expected_position).norm(), 0.0, 1e-6);
    ASSERT_NEAR(orientation.angularDistance(expected_orientation), 0.0, 1e-6);

    assert_fk_ik_round_trip(plugin, all_zeros);
}

// From the Fanuc official driver / descriptions.
// https://github.com/FANUC-CORPORATION/fanuc_description/blob/18d7c16/fanuc_crx_description/robot/crx10ia.urdf.xacro
TEST(CrxKinematicsPluginTest, test_plugin_crx10ia)
{
    const auto plugin = make_plugin("crx10ia", "flange");
    assert_no_ik_for_unreachable_pose(plugin);
    assert_all_zeros_pose(
        plugin, Eigen::Vector3d(0.7, -0.15, 0.245 + 0.54), Eigen::Quaterniond::Identity());
    for (int i = 0; i < 1000; ++i)
    {
        const auto joint_values = generate_random_joint_values();
        // print(joint_values);
        assert_fk_ik_round_trip(plugin, joint_values);
    }
}

// From the Fanuc official driver / descriptions.
// https://github.com/FANUC-CORPORATION/fanuc_description/blob/18d7c16/fanuc_crx_description/robot/crx30ia.urdf.xacro
TEST(CrxKinematicsPluginTest, test_plugin_crx30ia)
{
    const auto plugin = make_plugin("crx30ia", "flange");
    assert_no_ik_for_unreachable_pose(plugin);
    assert_all_zeros_pose(
        plugin, Eigen::Vector3d(0.93, -0.185, 0.37 + 0.95), Eigen::Quaterniond::Identity());
    for (int i = 0; i < 1000; ++i)
    {
        const auto joint_values = generate_random_joint_values();
        // print(joint_values);
        assert_fk_ik_round_trip(plugin, joint_values);
    }
}

// Custom URDF/SRDF with kinematic structure and tip frame differing from the offical ones.
// https://github.com/paolofrance/ros2_fanuc_interface/blob/c673a0d/crx_moveit_config/crx10ia_l_moveit_config/config/crx10ia_l.urdf.xacro
TEST(CrxKinematicsPluginTest, test_plugin_crx10ia_l_paolofrance)
{
    const auto plugin = make_plugin("crx10ia_l_paolofrance", "tcp");
    assert_no_ik_for_unreachable_pose(plugin);
    assert_all_zeros_pose(plugin,
                          Eigen::Vector3d(0.7, -0.15, 0.245 + 0.71),
                          Eigen::Quaterniond(/*w=*/0.0, 0.707107, 0.0, 0.707107));
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
