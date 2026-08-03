#pragma once
#include <rclcpp/version.h>

#if RCLCPP_VERSION_GTE(28, 1, 0)  // Jazzy or newer
#include <moveit/kinematics_base/kinematics_base.hpp>
#else
#include <moveit/kinematics_base/kinematics_base.h>
#endif
#include <Eigen/Core>

#if RCLCPP_VERSION_GTE(28, 1, 0)  // Jazzy or newer
#include <moveit/robot_state/robot_state.hpp>
#else
#include <moveit/robot_state/robot_state.h>
#endif

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

    /**
     * @brief Yoshikawa manipulability index, sqrt(det(J * J^T)), evaluated at the TCP.
     * Zero at a singularity. Accounts for any configured flange extension, since a longer
     * tool changes the translational part of the Jacobian.
     */
    double manipulability(const std::vector<double>& joint_values) const;

    /**
     * @brief Inverse condition number of the Jacobian, sigma_min / sigma_max, in [0, 1].
     * A scale-invariant alternative to manipulability(), analogous to TRAC-IK's Manip2.
     */
    double inverse_condition_number(const std::vector<double>& joint_values) const;

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

    /// How to pick a single solution out of the (up to 16) valid IK solutions.
    enum class SolutionSelection
    {
        /// L1 distance to the seed state. Upstream default; cheapest, best continuity.
        distance,
        /// Maximise sqrt(det(J * J^T)). Analogous to TRAC-IK's Manip1.
        manip1,
        /// Maximise sigma_min / sigma_max. Analogous to TRAC-IK's Manip2.
        manip2,
    };

  private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    crx_kinematics::CRXRobot robot_;
    std::array<double, 6> joint_limits_min_;
    std::array<double, 6> joint_limits_max_;
    double base_j1_height_;

    // Transform relating the URDFs "flange" frame orientation convention to the "Pendant" /
    // "Abbes and Poisson" convention. The latter is expected by CRXRobot::ik and CRXRobot::fk,
    // hence the need for conversion.
    Eigen::Isometry3d T_rostool_pendanttool_ = Eigen::Isometry3d::Identity();

    /**
     * @brief Fixed transform from the URDF tip frame to the actual tool control point, expressed
     * in the tip frame. Identity unless a flange extension / tool offset is configured, in which
     * case every pose crossing the plugin boundary refers to the TCP rather than the flange.
     */
    Eigen::Isometry3d T_rostool_tcp_ = Eigen::Isometry3d::Identity();

    // Cached for Jacobian evaluation. Both owned by robot_model_.
    const moveit::core::JointModelGroup* jmg_ = nullptr;
    const moveit::core::LinkModel* tip_link_ = nullptr;

    SolutionSelection solution_selection_ = SolutionSelection::distance;
    /// Solutions scoring below this on the active metric are rejected. Disabled when <= 0.
    double min_manipulability_ = 0.0;
    /// Weight on L1 seed distance, subtracted from the manipulability score. 0 disables.
    double seed_bias_ = 0.0;

    bool read_parameters(const rclcpp::Node::SharedPtr& node);

    // These take an already-constructed RobotState because building one costs far more than the
    // Jacobian itself, and DoIK evaluates up to 16 candidates per call.
    Eigen::MatrixXd jacobian(moveit::core::RobotState& state,
                             const std::vector<double>& joint_values) const;
    double manipulability(moveit::core::RobotState& state,
                          const std::vector<double>& joint_values) const;
    double inverse_condition_number(moveit::core::RobotState& state,
                                    const std::vector<double>& joint_values) const;
    bool extract_joint_limits_and_tcp_orientation();
    double score(moveit::core::RobotState& state,
                 const std::vector<double>& solution,
                 const std::vector<double>& reference_joint_values) const;
    bool respects_joint_limits(const std::vector<double>& solution) const;
    bool reproduces_desired_pose(const std::vector<double>& solution,
                                 const Eigen::Isometry3d& desired_pose) const;
};

}  // namespace crx_kinematics
