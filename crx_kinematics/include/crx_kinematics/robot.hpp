
#pragma once
#include <Eigen/Geometry>

namespace crx_kinematics
{

enum class RobotNameEnum
{
    crx5ia = 0,
    crx10ia,
    crx10ia_l,
    crx20ia_l,
    crx30ia
};

/**
 * @brief Holds the ("modified") DH params for a single joint.
 */
struct DHParams
{
    double a = 0;      // Translation along previous frames X-axis
    double alpha = 0;  // Rotation around previous frames X-axis
    double r = 0;      // Translation along this frames Z-axis
    double theta = 0;  // Rotation around this frames Z-axis

    /**
     * @brief Create the homogenous transformation matrix representing the pose of this joint wrt
     * its parent. In other words, the transform that takes data in the frame `L_i` and sends it to
     * the frame `L_{i-1}`.
     * @param joint_angle The angle of the joint.
     * @return The resulting transformation matrix. Or rather, a Isometry3d representing the same.
     */
    Eigen::Isometry3d T(const double joint_angle) const;
};

/**
 * @brief Holds the result of one "loop" of Step 2 -> Step 3 -> Step 4 of Abbes and Poisson (2024).
 * See Figure 5.
 */
struct CircleEvaluation
{
    /** @brief Constructor
     *  @param q The sample variable q ∈ [0, 2*pi] that defines this candidate O4 on the circle
     * (and remaining quantities that follow from this placement)
     *  @param T_R0_tool The desired or pose, i.e. the input to IK
     *  @param r4 The length of the forearm
     *  @param r5 The horizontal offset between the forearm and the flange
     *  @param r6 The distance between joint 5 and the flange
     *  @param a3 The length of the lower arm
     *  @param O5 The position of joint 5
     */
    explicit CircleEvaluation(const double q,
                              const Eigen::Isometry3d& T_R0_tool,
                              const double r4,
                              const double r5,
                              const double r6,
                              const double a3,
                              const Eigen::Vector3d& O5);

    double q;
    Eigen::Vector3d O4;
    bool triangle_inequality_holds;
    Eigen::Vector3d O3UP;
    Eigen::Vector3d O3DOWN;
    double dot_product_up;
    double dot_product_down;
};

class CRXRobot
{
  public:
    CRXRobot();
    CRXRobot(const RobotNameEnum& robot_name);
    Eigen::Isometry3d fk(const std::array<double, 6>& joint_values) const;
    std::vector<std::array<double, 6>> ik(const Eigen::Isometry3d& desired_pose) const;

  private:
    std::array<DHParams, 6> dh_params;
};

std::array<double, 6> to_xyzwpr(const Eigen::Isometry3d& T);
Eigen::Isometry3d from_xyzwpr(const std::array<double, 6>& xyzwpr);
std::array<double, 6> deg2rad(const std::array<double, 6>& joint_values);

}  // namespace crx_kinematics
