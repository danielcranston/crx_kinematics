#include "crx_kinematics/crx_kinematics_plugin.hpp"

#include <moveit/robot_model/robot_model.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace crx_kinematics
{
bool CRXKinematicsPlugin::initialize(rclcpp::Node::SharedPtr const& node,
                                     moveit::core::RobotModel const& robot_model,
                                     std::string const& group_name,
                                     std::string const& base_frame,
                                     std::vector<std::string> const& tip_frames,
                                     double search_discretization)
{
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);

    joint_names_ = jmg->getJointModelNames();
    link_names_.push_back(getTipFrame());

    RCLCPP_INFO(moveit::getLogger("crx_kinematics"), "tip_frame: '%s'", getTipFrame().c_str());
    return true;
}

bool CRXKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                        const std::vector<double>& ik_seed_state,
                                        std::vector<double>& solution,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    solution.clear();
    solution.push_back(0.1);
    for (std::size_t i = 0u; i < 5; ++i)
    {
        solution.push_back(0.0);
    }
    return true;
}

bool CRXKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state,
                                           double timeout,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    solution.clear();
    solution.push_back(0.1);
    for (std::size_t i = 0u; i < 5; ++i)
    {
        solution.push_back(0.0);
    }
    return true;
}

bool CRXKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state,
                                           double timeout,
                                           const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    solution.clear();
    solution.push_back(0.1);
    for (std::size_t i = 0u; i < 5; ++i)
    {
        solution.push_back(0.0);
    }
    return true;
}

bool CRXKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state,
                                           double timeout,
                                           std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    solution.clear();
    solution.push_back(0.1);
    for (std::size_t i = 0u; i < 5; ++i)
    {
        solution.push_back(0.0);
    }
    return true;
}

bool CRXKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state,
                                           double timeout,
                                           const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    solution.clear();
    solution.push_back(0.1);
    for (std::size_t i = 0u; i < 5; ++i)
    {
        solution.push_back(0.0);
    }
    return true;
}

bool CRXKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::msg::Pose>& poses) const
{
    poses.clear();
    for (std::size_t i = 0u; i < link_names.size(); ++i)
    {
        poses.push_back(geometry_msgs::msg::Pose());
    }
    return true;
}

bool CRXKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
                                        const std::vector<double>& ik_seed_state,
                                        std::vector<std::vector<double> >& solutions,
                                        kinematics::KinematicsResult& result,
                                        const kinematics::KinematicsQueryOptions& options) const
{
    solutions.clear();
    for (std::size_t i = 0u; i < ik_poses.size(); ++i)
    {
        solutions.push_back({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    }
    result.kinematic_error = kinematics::KinematicError::OK;
    result.solution_percentage = 1.0;
    return true;
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
