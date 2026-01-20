#include "crx_kinematics/crx_kinematics_plugin.hpp"

#include <algorithm>
#include <ranges>

#if RCLCPP_VERSION_GTE(28, 1, 0)  // Jazzy or newer
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#else
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#endif

#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace crx_kinematics
{
namespace
{
auto const LOGGER = rclcpp::get_logger("crx_kinematics");
}  // namespace

bool CRXKinematicsPlugin::initialize(rclcpp::Node::SharedPtr const& /*node*/,
                                     moveit::core::RobotModel const& robot_model,
                                     std::string const& group_name,
                                     std::string const& base_frame,
                                     std::vector<std::string> const& tip_frames,
                                     double search_discretization)
{
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

    if (tip_frames.size() != 1)
    {
        RCLCPP_INFO(LOGGER, "Only 1 tip frame is supported. Model has '%lu", tip_frames.size());
        return false;
    }

    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);

    joint_names_ = jmg->getJointModelNames();
    link_names_.push_back(getTipFrame());
    RCLCPP_INFO(LOGGER, "model_name: '%s'", robot_model.getName().c_str());

    RCLCPP_INFO(LOGGER, "model_frame: '%s'", robot_model.getModelFrame().c_str());
    RCLCPP_INFO(LOGGER, "tip_frame: '%s'", getTipFrame().c_str());
    RCLCPP_INFO(LOGGER, "base_frame: '%s'", base_frame_.c_str());
    for (const auto& name : joint_names_)
    {
        RCLCPP_DEBUG(LOGGER, "joint name: '%s'", name.c_str());
    }

    std::map<std::string, std::pair<crx_kinematics::RobotNameEnum, double>> model_map = {
        { "crx5ia", { crx_kinematics::RobotNameEnum::crx5ia, 0.185 } },
        { "crx10ia", { crx_kinematics::RobotNameEnum::crx10ia, 0.245 } },
        { "crx10ia_l", { crx_kinematics::RobotNameEnum::crx10ia_l, 0.245 } },
        { "crx20ia_l", { crx_kinematics::RobotNameEnum::crx20ia_l, 0.245 } },
        { "crx30ia", { crx_kinematics::RobotNameEnum::crx30ia, 0.37 } },
    };

    if (const auto& it = model_map.find(robot_model.getName()); it != model_map.end())
    {
        robot_ = crx_kinematics::CRXRobot(it->second.first, /*couple_j2_j3=*/false);
        base_j1_height_ = it->second.second;
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Unexpected model name: '%s'", robot_model.getName().c_str());
        return false;
    }

    return extract_joint_limits_and_tcp_orientation();
}

bool CRXKinematicsPlugin::DoIK(const geometry_msgs::msg::Pose& ik_pose,
                               std::vector<double>& solution,
                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                               const std::vector<double>& reference_joint_values,
                               const IKCallbackFn& solution_callback) const
{
    Eigen::Isometry3d T_R0_rostool;
    tf2::fromMsg(ik_pose, T_R0_rostool);

    // ROS pose is given in base frame, but CRXRobot::IK expects it in "R0" (pendant origin) frame
    T_R0_rostool.translation().z() = T_R0_rostool.translation().z() - base_j1_height_;

    // Account for the different definitions of the TCP frame between the Fanuc official URDFs and
    // Abbes and Poisson.
    const Eigen::Isometry3d T_R0_tool = T_R0_rostool * T_rostool_pendanttool_;

    // Find all IK solutions
    const std::vector<std::array<double, 6>> ik_solutions = robot_.ik(T_R0_tool);

    // Retain only valid IK solutions. A solution is valid if it fulfils the 3 following criteria:
    std::vector<std::vector<double>> valid_ik_solutions;
    valid_ik_solutions.reserve(ik_solutions.size());
    for (const auto& ik_sol : ik_solutions)
    {
        const auto ik_sol_vec = std::vector<double>(ik_sol.begin(), ik_sol.end());

        // 1: FK on the solution leads exactly to the desired pose
        if (!reproduces_desired_pose(ik_sol_vec, T_R0_rostool))
        {
            // In rare cases (~0.1%) some IK solutions can have a large error (up to ~3 cm).
            // The reason isn't known, but one likely culprit is numerical instabilities in
            // determine_joint_values.
            // I've yet to see any cases where orientation alone has had a large error, but that's
            // checked for as well (better safe than sorry).
            continue;
        }

        // 2: Respects joint limits
        if (!respects_joint_limits(ik_sol_vec))
        {
            continue;
        }

        // 3: Survives the user-defined callback (if provided)
        moveit_msgs::msg::MoveItErrorCodes error_code;
        solution_callback ? solution_callback(ik_pose, ik_sol_vec, error_code) :
                            [&error_code]() { error_code.val = error_code.SUCCESS; }();
        if (error_code.val == error_code.SUCCESS)
        {
            valid_ik_solutions.push_back(std::move(ik_sol_vec));
        }
    }

    RCLCPP_DEBUG(LOGGER, "solutions: %lu->%lu", ik_solutions.size(), valid_ik_solutions.size());

    if (valid_ik_solutions.empty())
    {
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }

    // Choose the IK solution closest to the seed (reference) state
    solution =
        reference_joint_values.size() != 6 ?
            valid_ik_solutions[0] :
            *std::ranges::min_element(
                valid_ik_solutions,  // https://en.cppreference.com/w/cpp/algorithm/ranges/min_element.html
                std::ranges::less{},
                // Projection
                [reference_joint_values](const auto& ik_sol) {
                    return std::abs(ik_sol[0] - reference_joint_values[0]) +  //
                           std::abs(ik_sol[1] - reference_joint_values[1]) +  //
                           std::abs(ik_sol[2] - reference_joint_values[2]) +  //
                           std::abs(ik_sol[3] - reference_joint_values[3]) +  //
                           std::abs(ik_sol[4] - reference_joint_values[4]) +  //
                           std::abs(ik_sol[5] - reference_joint_values[5]);
                });

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    return true;
}

bool CRXKinematicsPlugin::extract_joint_limits_and_tcp_orientation()
{
    int joints_traversed = 0;
    for (const auto& joint_name : joint_names_)
    {
        const urdf::JointConstSharedPtr joint = robot_model_->getURDF()->getJoint(joint_name);

        if (joint->type == urdf::Joint::REVOLUTE)
        {
            constexpr auto lowest = std::numeric_limits<double>::lowest();
            constexpr auto highest = std::numeric_limits<double>::max();

            const urdf::JointLimitsSharedPtr limits = joint->limits;
            joint_limits_min_[joints_traversed] = limits ? limits->lower : lowest;
            joint_limits_max_[joints_traversed] = limits ? limits->upper : highest;

            ++joints_traversed;
        }
    }

    if (joints_traversed != 6)
    {
        RCLCPP_ERROR(LOGGER, "Expected to find 6 revolute joints. Found %i", joints_traversed);
        return false;
    }

    // The internals of crx_kinematics::CRXRobot expects the tip to be "Z forward, X up" at rest
    const Eigen::Quaterniond pendanttool_orientation =
        Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    // Figure out what the tip orientation is for the URDF / planning group in use
    auto all_zero_state = moveit::core::RobotState(robot_model_);
    all_zero_state.setVariablePositions({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    const auto rostool_orientation =
        Eigen::Quaterniond(all_zero_state.getGlobalLinkTransform(getTipFrame()).linear());

    // Store the (purely rotational) transformation converting between the two orientations.
    T_rostool_pendanttool_.translation().setZero();
    T_rostool_pendanttool_.linear() =
        Eigen::Matrix3d(rostool_orientation.inverse() * pendanttool_orientation);

    return true;
}

bool CRXKinematicsPlugin::respects_joint_limits(const std::vector<double>& solution) const
{
    for (std::size_t i = 0; i < solution.size(); ++i)
    {
        if (solution[i] > joint_limits_max_[i] || solution[i] < joint_limits_min_[i])
        {
            return false;
        }
    }
    return true;
};

bool CRXKinematicsPlugin::reproduces_desired_pose(const std::vector<double>& solution,
                                                  const Eigen::Isometry3d& T_R0_rostool) const
{
    std::vector<geometry_msgs::msg::Pose> poses;
    getPositionFK({ getTipFrame() }, solution, poses);

    Eigen::Isometry3d T_R0_rostoolagain;
    tf2::fromMsg(poses[0], T_R0_rostoolagain);
    T_R0_rostoolagain.translation().z() -= base_j1_height_;

    const auto& p1 = T_R0_rostool.translation();
    const auto& p2 = T_R0_rostoolagain.translation();
    const auto q1 = Eigen::Quaterniond(T_R0_rostool.linear());
    const auto q2 = Eigen::Quaterniond(T_R0_rostoolagain.linear());

    const auto position_diff_squared = (p1 - p2).squaredNorm();
    const auto orientation_diff = q1.angularDistance(q2);
    if (position_diff_squared > 1e-6 * 1e-6 || orientation_diff > 0.01 * M_PI / 180)
    {
        return false;
    }
    return true;
}

// Virtual function override boilerplate below

bool CRXKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                        const std::vector<double>& ik_seed_state,
                                        std::vector<double>& solution,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& /*options*/) const
{
    return DoIK(ik_pose, solution, error_code, ik_seed_state);
}

bool CRXKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double /*timeout*/,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& /*options*/) const
{
    return DoIK(ik_pose, solution, error_code, ik_seed_state);
}

bool CRXKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double /*timeout*/,
    const std::vector<double>& /*consistency_limits*/,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& /*options*/) const
{
    return DoIK(ik_pose, solution, error_code, ik_seed_state);
}

bool CRXKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double /*timeout*/,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& /*options*/) const
{
    return DoIK(ik_pose, solution, error_code, ik_seed_state, solution_callback);
}

bool CRXKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double /*timeout*/,
    const std::vector<double>& /*consistency_limits*/,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& /*options*/) const
{
    return DoIK(ik_pose, solution, error_code, ik_seed_state, solution_callback);
}

bool CRXKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::msg::Pose>& poses) const
{
    if (!(link_names.size() == 1 && link_names[0] == getTipFrame()))
    {
        RCLCPP_ERROR(LOGGER,
                     "This plugin only supports looking up FK for the tip frame ('%s')",
                     getTipFrame().c_str());
        return false;
    }
    if (joint_angles.size() != 6)
    {
        RCLCPP_ERROR(LOGGER, "Expected 6 joint angles for FK, but got %lu", joint_angles.size());
        return false;
    }

    std::array<double, 6> joint_values;
    std::copy(joint_angles.begin(), joint_angles.end(), joint_values.begin());

    Eigen::Isometry3d T_R0_tool = robot_.fk(joint_values);

    // Translate to put the pose in base frame, not R0 frame.
    T_R0_tool.translation().z() += base_j1_height_;

    geometry_msgs::msg::Pose fk_pose = Eigen::toMsg(T_R0_tool * T_rostool_pendanttool_.inverse());

    poses.clear();
    poses.push_back(fk_pose);
    return true;
}

bool CRXKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::msg::Pose>& /*ik_poses*/,
                                        const std::vector<double>& /*ik_seed_state*/,
                                        std::vector<std::vector<double>>& /*solutions*/,
                                        kinematics::KinematicsResult& /*result*/,
                                        const kinematics::KinematicsQueryOptions& /*options*/) const
{
    return false;  // This is for robots with multiple tip links, not applicable to this plugin.
}

std::vector<std::string> const& CRXKinematicsPlugin::getJointNames() const
{
    return joint_names_;
}

std::vector<std::string> const& CRXKinematicsPlugin::getLinkNames() const
{
    return link_names_;
}
}  // namespace crx_kinematics

PLUGINLIB_EXPORT_CLASS(crx_kinematics::CRXKinematicsPlugin, kinematics::KinematicsBase);
