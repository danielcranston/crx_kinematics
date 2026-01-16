#pragma once
#include <moveit/kinematics_base/kinematics_base.hpp>

#include "crx_kinematics/robot.hpp"

namespace crx_kinematics
{

class CRXKinematicsPlugin : public kinematics::KinematicsBase
{
  public:
    virtual bool initialize(rclcpp::Node::SharedPtr const& node,
                            moveit::core::RobotModel const& robot_model,
                            std::string const& group_name,
                            std::string const& base_frame,
                            std::vector<std::string> const& tip_frames,
                            double search_discretization) override final;

    bool DoIK(const geometry_msgs::msg::Pose& ik_pose,
              std::vector<double>& solution,
              moveit_msgs::msg::MoveItErrorCodes& error_code,
              const std::vector<double>& reference_joint_values,
              const IKCallbackFn& solution_callback = IKCallbackFn()) const;

    // Virtual function override boilerplate below

    virtual bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                               const std::vector<double>& ik_seed_state,
                               std::vector<double>& solution,
                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                               const kinematics::KinematicsQueryOptions& options =
                                   kinematics::KinematicsQueryOptions()) const override final;

    virtual bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                  const std::vector<double>& ik_seed_state,
                                  double timeout,
                                  std::vector<double>& solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  const kinematics::KinematicsQueryOptions& options =
                                      kinematics::KinematicsQueryOptions()) const override final;
    virtual bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                  const std::vector<double>& ik_seed_state,
                                  double timeout,
                                  const std::vector<double>& consistency_limits,
                                  std::vector<double>& solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  const kinematics::KinematicsQueryOptions& options =
                                      kinematics::KinematicsQueryOptions()) const override final;
    virtual bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                  const std::vector<double>& ik_seed_state,
                                  double timeout,
                                  std::vector<double>& solution,
                                  const IKCallbackFn& solution_callback,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  const kinematics::KinematicsQueryOptions& options =
                                      kinematics::KinematicsQueryOptions()) const override final;
    virtual bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                  const std::vector<double>& ik_seed_state,
                                  double timeout,
                                  const std::vector<double>& consistency_limits,
                                  std::vector<double>& solution,
                                  const IKCallbackFn& solution_callback,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  const kinematics::KinematicsQueryOptions& options =
                                      kinematics::KinematicsQueryOptions()) const override final;

    virtual bool getPositionFK(const std::vector<std::string>& link_names,
                               const std::vector<double>& joint_angles,
                               std::vector<geometry_msgs::msg::Pose>& poses) const override final;

    virtual bool getPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
                               const std::vector<double>& ik_seed_state,
                               std::vector<std::vector<double> >& solutions,
                               kinematics::KinematicsResult& result,
                               const kinematics::KinematicsQueryOptions& options =
                                   kinematics::KinematicsQueryOptions()) const override final;

    virtual const std::vector<std::string>& getJointNames() const override final;
    virtual const std::vector<std::string>& getLinkNames() const override final;

  private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    crx_kinematics::CRXRobot robot_;
    std::array<double, 6> joint_limits_min_;
    std::array<double, 6> joint_limits_max_;
    double base_j1_height_;

    bool extract_joint_limits();
    bool respects_joint_limits(const std::vector<double>& solution) const;
};

}  // namespace crx_kinematics
