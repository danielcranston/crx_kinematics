#include "crx_kinematics/crx_kinematics_plugin.hpp"

#include <moveit/robot_model/robot_model.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace crx_kinematics
{
// Transform relating the ROS driver "flange" frame orientation convention to the "Pendant" / "Abbes
// and Poisson" convention. The latter is expected by CRXRobot::ik, hence the need for conversion.
const Eigen::Isometry3d T_rostool_pendanttool = []() {
    Eigen::Matrix4d mat = Eigen::Vector4d(-1.0, -1, 1, 1).asDiagonal().toDenseMatrix();
    Eigen::Isometry3d T;
    T.matrix() = mat;
    return T;
}();

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
    RCLCPP_INFO(moveit::getLogger("crx_kinematics"), "base_frame: '%s'", base_frame_.c_str());
    for (const auto& name : joint_names_)
    {
        RCLCPP_INFO(moveit::getLogger("crx_kinematics"), "joint name: '%s'", name.c_str());
    }

    robot_ = crx_kinematics::CRXRobot();
    return true;
}

bool CRXKinematicsPlugin::DoIK(const geometry_msgs::msg::Pose& ik_pose,
                               std::vector<double>& solution,
                               moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
    Eigen::Isometry3d T_R0_rostool;
    tf2::fromMsg(ik_pose, T_R0_rostool);

    // ROS pose is given in base frame, but CRXRobot::IK expects it in "R0" (pendant origin) frame
    T_R0_rostool.translation().z() = T_R0_rostool.translation().z() - 0.245;

    // Account for the different definitions of the TCP frame between the Fanuc official URDFs and
    // Abbes and Poisson.
    const Eigen::Isometry3d T_R0_tool = T_R0_rostool * T_rostool_pendanttool;

    const std::vector<std::array<double, 6>> ik_solutions = robot_.ik(T_R0_tool);

    if (ik_solutions.empty())
    {
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }

    const auto& ik_solution = ik_solutions[0];  // Arbitrarily

    solution.clear();
    solution.push_back(ik_solution[0]);
    solution.push_back(ik_solution[1]);
    solution.push_back(ik_solution[2] + ik_solution[1]);  // "Undo" the J2/J3 coupling
    solution.push_back(ik_solution[3]);
    solution.push_back(ik_solution[4]);
    solution.push_back(ik_solution[5]);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    return true;
}

// Virtual function override boilerplate below

bool CRXKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                        const std::vector<double>& ik_seed_state,
                                        std::vector<double>& solution,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
    return DoIK(ik_pose, solution, error_code);
}

bool CRXKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state,
                                           double timeout,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
    return DoIK(ik_pose, solution, error_code);
}

bool CRXKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state,
                                           double timeout,
                                           const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
    return DoIK(ik_pose, solution, error_code);
}

bool CRXKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state,
                                           double timeout,
                                           std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
    return DoIK(ik_pose, solution, error_code);
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
    return DoIK(ik_pose, solution, error_code);
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
                                        std::vector<std::vector<double>>& solutions,
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
