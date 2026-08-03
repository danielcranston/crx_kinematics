#include <algorithm>
#include <cmath>
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

Eigen::Isometry3d make_isometry(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat)
{
    auto pose = Eigen::Isometry3d();
    pose.translation() = pos;
    pose.linear() = Eigen::Matrix3d(quat);
    return pose;
}

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
        random_joint_value(-60, 60),    /* Ensure we stay away from the edge of the robot envelope
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

crx_kinematics::CRXKinematicsPlugin
make_plugin(const std::string& robot_name,
            const std::string tip_frame = "flange",
            const std::vector<rclcpp::Parameter>& parameter_overrides = {})
{
    // Parameters are namespaced exactly as MoveIt namespaces kinematics.yaml, so the overrides
    // below exercise the same code path as a real launch.
    auto options = rclcpp::NodeOptions().parameter_overrides(parameter_overrides);
    auto node = rclcpp::Node::make_shared("test_crx_kinematics_plugin", options);
    auto robot_model = load_crx_robot_model(robot_name);

    auto plugin = crx_kinematics::CRXKinematicsPlugin();
    if (!plugin.initialize(node, *robot_model, "manipulator", "base_link", { tip_frame }, 0.0))
    {
        throw std::runtime_error("Could not initialize plugin for " + robot_name);
    }

    return plugin;
}

rclcpp::Parameter kinematics_param(const std::string& name, const rclcpp::ParameterValue& value)
{
    return rclcpp::Parameter("robot_description_kinematics.manipulator." + name, value);
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
    ASSERT_NEAR(q1.angularDistance(q2), 0.0, 1e-5) << to_str(joint_values);
}

void assert_all_zeros_pose(const crx_kinematics::CRXKinematicsPlugin& plugin,
                           const Eigen::Isometry3d& all_zero_pose)
{
    const std::vector<double> all_zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    const geometry_msgs::msg::Pose fk_pose = do_fk(plugin, all_zeros);
    const auto [position, orientation] = pose_to_eigen(fk_pose);
    const auto [expected_position, expected_orientation] = [&all_zero_pose]() {
        return std::make_pair(all_zero_pose.translation(),
                              Eigen::Quaterniond(all_zero_pose.linear()));
    }();

    ASSERT_NEAR((position - expected_position).norm(), 0.0, 1e-6);
    ASSERT_NEAR(orientation.angularDistance(expected_orientation), 0.0, 1e-6);

    assert_fk_ik_round_trip(plugin, all_zeros);
}

void test_robot(const std::string& robot_name,
                const std::string& tip_frame,
                const Eigen::Isometry3d& all_zero_pose)
{
    const auto plugin = make_plugin(robot_name, tip_frame);
    assert_no_ik_for_unreachable_pose(plugin);
    assert_all_zeros_pose(plugin, all_zero_pose);
    for (int i = 0; i < 1000; ++i)
    {
        const auto joint_values = generate_random_joint_values();
        // print(joint_values);
        assert_fk_ik_round_trip(plugin, joint_values);
    }
}

// From the Fanuc official driver / descriptions.
// https://github.com/FANUC-CORPORATION/fanuc_description/blob/v1.2.2/fanuc_crx_description/robot/crx3ia.urdf.xacro
TEST(CrxKinematicsPluginTest, test_plugin_crx3ia)
{
    test_robot(
        "crx3ia",
        "flange",
        make_isometry(Eigen::Vector3d(0.403, -0.111, 0.441), Eigen::Quaterniond::Identity()));
}

// From the Fanuc official driver / descriptions.
// https://github.com/FANUC-CORPORATION/fanuc_description/blob/v1.2.2/fanuc_crx_description/robot/crx5ia.urdf.xacro
TEST(CrxKinematicsPluginTest, test_plugin_crx5ia)
{
    test_robot("crx5ia",
               "flange",
               make_isometry(Eigen::Vector3d(0.575, -0.13, 0.595), Eigen::Quaterniond::Identity()));
}

// From the Fanuc official driver / descriptions.
// https://github.com/FANUC-CORPORATION/fanuc_description/blob/v1.2.2/fanuc_crx_description/robot/crx10ia.urdf.xacro
TEST(CrxKinematicsPluginTest, test_plugin_crx10ia)
{
    test_robot(
        "crx10ia",
        "flange",
        make_isometry(Eigen::Vector3d(0.7, -0.15, 0.245 + 0.54), Eigen::Quaterniond::Identity()));
}

// From the Fanuc official driver / descriptions.
// https://github.com/FANUC-CORPORATION/fanuc_description/blob/v1.2.2/fanuc_crx_description/robot/crx30ia.urdf.xacro
TEST(CrxKinematicsPluginTest, test_plugin_crx30ia)
{
    test_robot(
        "crx30ia",
        "flange",
        make_isometry(Eigen::Vector3d(0.93, -0.185, 0.37 + 0.95), Eigen::Quaterniond::Identity()));
}

// Custom URDF/SRDF with kinematic structure and tip frame differing from the offical ones.
// https://github.com/paolofrance/ros2_fanuc_interface/blob/c673a0d/crx_moveit_config/crx10ia_l_moveit_config/config/crx10ia_l.urdf.xacro
TEST(CrxKinematicsPluginTest, test_plugin_crx10ia_l_paolofrance)
{
    test_robot("crx10ia_l_paolofrance",
               "tcp",
               make_isometry(Eigen::Vector3d(0.7, -0.15, 0.245 + 0.71),
                             Eigen::Quaterniond(/*w=*/0.0, 0.707107, 0.0, 0.707107)));
}

// The tip frame of crx10ia has identity orientation at the all-zeros state, so an extension
// along the tip frame's +Z axis shows up directly as a shift in world Z.
TEST(CrxKinematicsPluginTest, flange_extension_shifts_the_reported_tcp)
{
    constexpr double extension = 0.15;

    const auto plain = make_plugin("crx10ia", "flange");
    const auto extended = make_plugin(
        "crx10ia",
        "flange",
        { kinematics_param("flange_extension", rclcpp::ParameterValue(extension)) });

    const std::vector<double> all_zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    const auto [plain_position, plain_orientation] = pose_to_eigen(do_fk(plain, all_zeros));
    const auto [ext_position, ext_orientation] = pose_to_eigen(do_fk(extended, all_zeros));

    ASSERT_NEAR((ext_position - (plain_position + Eigen::Vector3d(0.0, 0.0, extension))).norm(),
                0.0,
                1e-9);
    // A pure translation of the TCP must not change its orientation.
    ASSERT_NEAR(ext_orientation.angularDistance(plain_orientation), 0.0, 1e-9);
}

// The important regression: IK must undo the extension it applied in FK, otherwise every pose
// sent to the plugin is silently off by the tool length.
TEST(CrxKinematicsPluginTest, flange_extension_round_trips)
{
    const auto plugin = make_plugin(
        "crx10ia",
        "flange",
        { kinematics_param("flange_extension", rclcpp::ParameterValue(0.15)) });

    assert_no_ik_for_unreachable_pose(plugin);
    for (int i = 0; i < 200; ++i)
    {
        assert_fk_ik_round_trip(plugin, generate_random_joint_values());
    }
}

TEST(CrxKinematicsPluginTest, manipulability_is_well_formed)
{
    const auto plugin = make_plugin("crx10ia", "flange");

    for (int i = 0; i < 50; ++i)
    {
        const auto joint_values = generate_random_joint_values();

        const double manip = plugin.manipulability(joint_values);
        ASSERT_TRUE(std::isfinite(manip)) << to_str(joint_values);
        ASSERT_GE(manip, 0.0) << to_str(joint_values);

        const double inverse_condition = plugin.inverse_condition_number(joint_values);
        ASSERT_TRUE(std::isfinite(inverse_condition)) << to_str(joint_values);
        ASSERT_GE(inverse_condition, 0.0) << to_str(joint_values);
        ASSERT_LE(inverse_condition, 1.0) << to_str(joint_values);
    }
}

// Guards against the Jacobian reference point being left at the tip link origin, which would make
// the metric blind to the tool and silently wrong for any extended flange.
TEST(CrxKinematicsPluginTest, manipulability_accounts_for_the_flange_extension)
{
    const auto plain = make_plugin("crx10ia", "flange");
    const auto extended = make_plugin(
        "crx10ia",
        "flange",
        { kinematics_param("flange_extension", rclcpp::ParameterValue(0.5)) });

    const std::vector<double> joint_values = { 0.3, -0.4, 0.6, 0.5, 0.9, -0.2 };

    ASSERT_GT(std::abs(plain.manipulability(joint_values) - extended.manipulability(joint_values)),
              1e-9);
}

TEST(CrxKinematicsPluginTest, manipulability_selection_still_returns_valid_solutions)
{
    for (const std::string& selection : { "manip1", "manip2" })
    {
        const auto plugin = make_plugin(
            "crx10ia",
            "flange",
            { kinematics_param("solution_selection", rclcpp::ParameterValue(selection)) });

        assert_no_ik_for_unreachable_pose(plugin);
        for (int i = 0; i < 200; ++i)
        {
            assert_fk_ik_round_trip(plugin, generate_random_joint_values());
        }
    }
}

// With a manipulability floor in place, whatever survives must clear that floor.
TEST(CrxKinematicsPluginTest, min_manipulability_filters_solutions)
{
    constexpr double floor_value = 0.02;

    const auto plugin = make_plugin(
        "crx10ia",
        "flange",
        { kinematics_param("solution_selection", rclcpp::ParameterValue(std::string("manip1"))),
          kinematics_param("min_manipulability", rclcpp::ParameterValue(floor_value)) });

    int solved = 0;
    for (int i = 0; i < 200; ++i)
    {
        const auto joint_values = generate_random_joint_values();
        const geometry_msgs::msg::Pose fk_pose = do_fk(plugin, joint_values);

        moveit_msgs::msg::MoveItErrorCodes error_code;
        std::vector<double> solution;
        plugin.getPositionIK(fk_pose, joint_values, solution, error_code);

        if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            ++solved;
            ASSERT_GE(plugin.manipulability(solution), floor_value) << to_str(joint_values);
        }
    }

    // Sanity check that the floor did not simply reject everything.
    ASSERT_GT(solved, 0);
}

TEST(CrxKinematicsPluginTest, unknown_solution_selection_fails_initialization)
{
    ASSERT_THROW(make_plugin("crx10ia",
                             "flange",
                             { kinematics_param("solution_selection",
                                                rclcpp::ParameterValue(std::string("nonsense"))) }),
                 std::runtime_error);
}

int main(int argc, char** argv)
{
    rclcpp::init(0, nullptr);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
