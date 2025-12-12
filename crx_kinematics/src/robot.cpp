#include "crx_kinematics/robot.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>

namespace crx_kinematics
{
namespace
{

/**
 * @brief Corresponds to each column in Table 2 of Abbes and Poisson (2024).
 * Note that some rows annoyingly attribute values to i-1 while some attribute to i.
 * The below params attribute all values in each column to i for convenience, although it's
 * technically incorrect according to the modified DH convention.
 */
std::array<DHParams, 6>
get_dh_params(const double l_2, const double l_4, const double l_5, const double l_6)
{
    DHParams L1 = {};
    DHParams L2 = { .alpha = -M_PI / 2, .theta = -M_PI / 2 };
    DHParams L3 = { .a = l_2, .alpha = M_PI };
    DHParams L4 = { .alpha = -M_PI / 2, .r = -l_4 };
    DHParams L5 = { .alpha = M_PI / 2, .r = l_5 };
    DHParams L6 = { .alpha = -M_PI / 2, .r = -l_6 };

    return { L1, L2, L3, L4, L5, L6 };
}

std::array<DHParams, 6> crx5ia_params()
{
    return get_dh_params(0.41, 0.43, 0.13, 0.145);
}

std::array<DHParams, 6> crx10ia_params()
{
    return get_dh_params(0.54, 0.54, 0.15, 0.16);
}

std::array<DHParams, 6> crx10ia_l_params()
{
    return get_dh_params(0.71, 0.54, 0.15, 0.16);
}

std::array<DHParams, 6> crx20ia_l_params()
{
    return crx10ia_l_params();
}

std::array<DHParams, 6> crx30ia_params()
{
    return get_dh_params(0.95, 0.75, 0.185, 0.18);
}

std::array<DHParams, 6> get_dh_params(const RobotNameEnum& robot_name)
{
    switch (robot_name)
    {
        case RobotNameEnum::crx5ia:
            return crx5ia_params();
        case RobotNameEnum::crx10ia:
            return crx10ia_params();
        case RobotNameEnum::crx10ia_l:
            return crx10ia_l_params();
        case RobotNameEnum::crx20ia_l:
            return crx20ia_l_params();
        case RobotNameEnum::crx30ia:
            return crx30ia_params();
    }

    // The compiler will complain if the above switch is non-exhaustive.
    // This line is essentially only to guard against static_cast shenanigans.
    return crx5ia_params();
}

const Eigen::Matrix4d T_J6_tool = []() {
    return Eigen::Vector4d(1.0, -1, -1, 1).asDiagonal().toDenseMatrix();
}();

/**
 * @brief Returns the vector rejection of a from b.
 * In other words, the orthogonal component of a that is perpendicular to b.
 * See https://en.wikipedia.org/wiki/Vector_projection
 */
Eigen::Vector3d vector_rejection(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    const Eigen::Vector3d projection = a.dot(b) / b.dot(b) * b;
    return a - projection;
}

/**
 * @brief Construct the plane O0O3O4
 * See Step 3 of Abbes and Poisson (2024)
 */
Eigen::Isometry3d construct_plane(const Eigen::Vector3d& dir1, const Eigen::Vector3d& dir2)
{
    const Eigen::Vector3d x = dir1.normalized();
    const Eigen::Vector3d y = vector_rejection(dir2, dir1).normalized();
    const Eigen::Vector3d z = x.cross(y);

    Eigen::Matrix3d R = Eigen::Matrix3d();
    R << x, y, z;

    Eigen::Isometry3d out;
    out.translation().setZero();
    out.linear() = R;
    return out;
}

/**
 * @brief Given a triangle where
      1. The lengths of each side are known,
      2. The first corner is at (0, 0)
      3. The second corner is at (AB, 0)
    Returns the position (x, y) of the third triangle.
    Note that flipping the sign of y also is a valid solution, hence both solutions are returned

           C = (x, y)
         /   \
      /        \
    A ---------- B = (AB, 0)

    https://math.stackexchange.com/a/544025
 */
std::pair<Eigen::Vector2d, Eigen::Vector2d> find_third_triangle_corner(const double AB,
                                                                       const double AC,
                                                                       const double BC)
{
    const double x = (AC * AC - BC * BC + AB * AB) / (2 * AB);

    if (x * x > AC * AC)
    {
        // Triangle inequality doesn't hold.
        return std::make_pair(Eigen::Vector2d(x, 0), Eigen::Vector2d(x, 0));
    }
    const double y = std::sqrt((AC * AC) - (x * x));

    return std::make_pair(Eigen::Vector2d(x, y), Eigen::Vector2d(x, -y));
}

/**
 * @brief Finds the root of a real function f given an interval [a, b] where f(a) and f(b) have
 * different signs. This function implements the bisection method:
 * https://en.wikipedia.org/wiki/Bisection_method
 * @param a One side of the interval
 * @param b The other side of the interval
 * @param f The function to find the root of. In our context, this function takes a double q and
 * returns one of the "dot product" values of a CircleEvaluation (corresponding to O3UP or O3DOWN)
 * @param fa The function value evaluated at a
 * @param fb The function value evaluated at b
 * @param tolerance Dictates when a found q is deemed acceptably close to zero.
 * @param max_iterations Dictates the max number of iterations to perform in case a acceptable
 * solution is not found
 * @return The found root of the function
 */
double find_zero(double a,
                 double b,
                 std::function<double(double)> f,
                 double fa,
                 double fb,
                 const double tolerance = 1e-6,
                 const int max_iterations = 100)
{
    if (std::fabs(fa) < tolerance)
    {
        return a;
    }
    if (std::fabs(fb) < tolerance)
    {
        return b;
    }

    if (fa < fb)
    {
        std::swap(a, b);
        std::swap(fa, fb);
    }

    double q = 0.5 * (a + b);
    double dot_product = f(q);

    int num_iterations = 1;

    while (std::fabs(dot_product) >= tolerance)
    {
        if (num_iterations == max_iterations)
        {
            break;
        }

        if (dot_product >= 0.0)
        {
            a = q;
            fa = dot_product;
        }
        else
        {
            b = q;
            fb = dot_product;
        }

        q = 0.5 * (a + b);
        dot_product = f(q);

        ++num_iterations;
    }

    return q;
}

/**
 * @brief Given valid placements of the origins of J3 and J4, the values of each joint can be
 * computed. This function does that.
 * See Step 6 of Abbes and Poisson 2024.
 * @return The IK solution for this configuration of O3 and O4.
 */
std::array<double, 6> determine_joint_values(const Eigen::Vector3d& O3,
                                             const Eigen::Vector3d& O4,
                                             const Eigen::Vector3d& O5,
                                             const Eigen::Isometry3d& T_R0_tool,
                                             const std::array<DHParams, 6>& dh_params)
{
    const auto O6 = T_R0_tool.translation();
    const double J1 = std::atan2(O4.y(), O4.x());

    const auto T_L1_L0 = dh_params[0].T(J1).inverse();
    const auto R_L1_L0 = T_L1_L0.linear();
    const auto O_1_3 = R_L1_L0 * O3;
    const double J2 = std::atan2(O_1_3.x(), O_1_3.z());

    const auto O_1_4 = R_L1_L0 * O4;
    const double J3 = std::atan2(O_1_4.z() - O_1_3.z(), O_1_4.x() - O_1_3.x());

    const auto T_L2_L1 = dh_params[1].T(J2).inverse();
    const auto T_L3_L2 = dh_params[2].T(J2 + J3).inverse();  // Handle Fanuc J2/J3 coupling

    const auto T_L3_L0 = T_L3_L2 * T_L2_L1 * T_L1_L0;
    const auto O_3_5 = T_L3_L0 * O5;
    const double J4 = std::atan2(O_3_5.x(), O_3_5.z());

    const auto T_L4_L3 = dh_params[3].T(J4).inverse();
    const auto T_L4_L0 = T_L4_L3 * T_L3_L0;
    const auto O_4_6 = T_L4_L0 * O6;
    const double J5 = std::atan2(O_4_6.x(), -O_4_6.z());

    const auto T_L5_L0 = dh_params[4].T(J5).inverse() * T_L4_L0;
    const auto T_L5_tool = T_L5_L0 * T_R0_tool;
    const auto tool_x_in_L5 = T_L5_tool.linear().col(0);
    const double J6 = std::atan2(-tool_x_in_L5.z(), tool_x_in_L5.x());

    return { J1, J2, J3, J4, J5, J6 };
}

double harmonize_towards_zero(const double x)
{
    if (x < -M_PI)
    {
        return x + 2.0 * M_PI;
    }
    if (x > M_PI)
    {
        return x - 2.0 * M_PI;
    }
    return x;
};

}  // namespace

CircleEvaluation::CircleEvaluation(const double q,
                                   const Eigen::Isometry3d& T_R0_tool,
                                   const double r4,
                                   const double r5,
                                   const double r6,
                                   const double a3,
                                   const Eigen::Vector3d& O5)
  : q(q)
{
    O4 = T_R0_tool * Eigen::Vector3d(r5 * std::cos(q), r5 * std::sin(q), r6);
    triangle_inequality_holds = O4.norm() <= std::abs(r4) + std::abs(a3);

    const Eigen::Isometry3d T_R0_plane = construct_plane(O4, Eigen::Vector3d::UnitZ());
    const auto [corner_up, corner_down] = find_third_triangle_corner(O4.norm(), a3, -r4);

    O3UP = T_R0_plane * Eigen::Vector3d(corner_up.x(), corner_up.y(), 0.0);
    O3DOWN = T_R0_plane * Eigen::Vector3d(corner_down.x(), corner_down.y(), 0.0);

    const Eigen::Vector3d Z5 = (O5 - O4).normalized();
    dot_product_up = Eigen::Vector3d(O4 - O3UP).normalized().dot(Z5);
    dot_product_down = Eigen::Vector3d(O4 - O3DOWN).normalized().dot(Z5);
}

Eigen::Isometry3d DHParams::T(double joint_value) const
{
    double stheta = std::sin(joint_value + theta);
    double ctheta = std::cos(joint_value + theta);
    double salpha = std::sin(alpha);
    double calpha = std::cos(alpha);

    Eigen::Matrix4d T_parent_child;
    // Matching Eq 1. of Abbes and Poisson 2024
    T_parent_child << ctheta, -stheta, 0, a,                     //
        stheta * calpha, ctheta * calpha, -salpha, -r * salpha,  //
        stheta * salpha, ctheta * salpha, calpha, r * calpha,    //
        0, 0, 0, 1;

    Eigen::Isometry3d out;
    out.matrix() = T_parent_child;
    return out;
}

CRXRobot::CRXRobot() : dh_params(crx10ia_params())
{
}

CRXRobot::CRXRobot(const RobotNameEnum& robot_name) : dh_params(get_dh_params(robot_name))
{
}

Eigen::Isometry3d CRXRobot::fk(const std::array<double, 6>& joint_values) const
{
    Eigen::Isometry3d out = dh_params[0].T(joint_values[0]);
    out = out * dh_params[1].T(joint_values[1]);
    out = out * dh_params[2].T(joint_values[2] + joint_values[1]);  // Handle Fanuc J2/J3 coupling
    out = out * dh_params[3].T(joint_values[3]);
    out = out * dh_params[4].T(joint_values[4]);
    out = out * dh_params[5].T(joint_values[5]);
    out = out * T_J6_tool;
    return out;
}

std::vector<std::array<double, 6>> CRXRobot::ik(const Eigen::Isometry3d& desired_pose) const
{
    // Step 1
    const Eigen::Vector3d O5 =
        desired_pose /*aka T_R0_tool*/ * Eigen::Vector3d(0.0, 0.0, dh_params[5].r);

    const double r4 = dh_params[3].r;
    const double r5 = dh_params[4].r;
    const double r6 = dh_params[5].r;
    const double a3 = dh_params[2].a;

    auto make_circle_evaluation = [&desired_pose, r4, r5, r6, a3, &O5](double q) {
        return CircleEvaluation(q, desired_pose, r4, r5, r6, a3, O5);
    };

    auto f_up = [&make_circle_evaluation](double q) {
        return make_circle_evaluation(q).dot_product_up;
    };
    auto f_down = [&make_circle_evaluation](double q) {
        return make_circle_evaluation(q).dot_product_down;
    };

    auto [previous_up_dot, previous_down_dot] = [&make_circle_evaluation]() {
        const auto ce = make_circle_evaluation(0.0);
        return std::make_pair(ce.dot_product_up, ce.dot_product_down);
    }();

    // Step 2, 3, 4, 5 and 6
    std::vector<std::array<double, 6>> solutions;
    solutions.reserve(16);
    for (int q_deg = 1; q_deg < 360; ++q_deg)
    {
        auto circle_evaluation = CircleEvaluation(
            static_cast<double>(q_deg) / 180.0 * M_PI, desired_pose, r4, r5, r6, a3, O5);

        const double up_dot = circle_evaluation.dot_product_up;
        const double down_dot = circle_evaluation.dot_product_down;

        if (std::signbit(up_dot) != std::signbit(previous_up_dot))
        {
            const double root = find_zero(static_cast<double>(q_deg - 1) / 180.0 * M_PI,
                                          static_cast<double>(q_deg) / 180.0 * M_PI,
                                          f_up,
                                          previous_up_dot,
                                          up_dot);
            const CircleEvaluation ce = CircleEvaluation(root, desired_pose, r4, r5, r6, a3, O5);
            if (ce.triangle_inequality_holds)
            {
                solutions.push_back(
                    determine_joint_values(ce.O3UP, ce.O4, O5, desired_pose, dh_params));
            }
        }
        if (std::signbit(down_dot) != std::signbit(previous_down_dot))
        {
            const double root = find_zero(static_cast<double>(q_deg - 1) / 180.0 * M_PI,
                                          static_cast<double>(q_deg) / 180.0 * M_PI,
                                          f_down,
                                          previous_down_dot,
                                          down_dot);
            const CircleEvaluation ce = CircleEvaluation(root, desired_pose, r4, r5, r6, a3, O5);
            if (ce.triangle_inequality_holds)
            {
                solutions.push_back(
                    determine_joint_values(ce.O3DOWN, ce.O4, O5, desired_pose, dh_params));
            }
        }

        previous_up_dot = up_dot;
        previous_down_dot = down_dot;
    }

    // Step 7
    const auto num_regular_solutions = solutions.size();
    for (auto i = 0u; i < num_regular_solutions; ++i)
    {
        const auto& sol = solutions[i];
        std::array<double, 6> dual = {
            harmonize_towards_zero(sol[0] - M_PI),
            harmonize_towards_zero(-sol[1]),
            harmonize_towards_zero(M_PI - sol[2]),
            harmonize_towards_zero(sol[3] - M_PI),
            sol[4],
            sol[5],
        };
        solutions.push_back(dual);
    }

    return solutions;
}

std::array<double, 6> to_xyzwpr(const Eigen::Isometry3d& T)
{
    const Eigen::Vector3d xyz = T.translation() * 1000.0;

    const Eigen::Matrix3d R = T.linear();
    // https://github.com/matthew-brett/transforms3d/blob/dc877e1/transforms3d/euler.py#L237
    const double cy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    Eigen::Vector3d wpr{};
    if (cy > std::numeric_limits<double>::epsilon() * 4.0)
    {
        wpr = Eigen::Vector3d(std::atan2(R(2, 1), R(2, 2)) * 180.0 / M_PI,
                              std::atan2(-R(2, 0), cy) * 180.0 / M_PI,
                              std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI);
    }
    else
    {
        wpr = Eigen::Vector3d(std::atan2(-R(1, 2), R(1, 1)) * 180.0 / M_PI,
                              std::atan2(-R(2, 0), cy) * 180.0 / M_PI,
                              0.0);
    }

    return { xyz[0], xyz[1], xyz[2], wpr[0], wpr[1], wpr[2] };
}

Eigen::Isometry3d from_xyzwpr(const std::array<double, 6>& xyzwpr)
{
    // xyzwpr expressed in millimeters and degrees, for convenience
    Eigen::Isometry3d T;
    T.translation().x() = xyzwpr[0] / 1000.0;
    T.translation().y() = xyzwpr[1] / 1000.0;
    T.translation().z() = xyzwpr[2] / 1000.0;

    Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(xyzwpr[5] / 180.0 * M_PI, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(xyzwpr[4] / 180.0 * M_PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(xyzwpr[3] / 180.0 * M_PI, Eigen::Vector3d::UnitX());

    T.linear() = Eigen::Matrix3d(rotation);
    return T;
}

std::array<double, 6> deg2rad(const std::array<double, 6>& joint_values)
{
    std::array<double, 6> out{};
    for (auto i = 0u; i < joint_values.size(); ++i)
    {
        out[i] = joint_values[i] / 180.0 * M_PI;
    }
    return out;
};

}  // namespace crx_kinematics
