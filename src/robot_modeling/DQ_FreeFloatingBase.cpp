/**
    This file is not (currently) part of the official release or development branches of DQ Robotics.
    This file incorporates elements of the official releases, but was written by Daniel S. Johnson
    without review by the original developers. As such, any problems introduced are my own responsibility.

    In accordance with the licencing arrangements for DQ Robotics itself, this file is distributed under
    an LGPL-3.0 license.

    As with all files in this fork, this file should be considered a work in progress. Some functions have not
    been properly implemented yet, and these will raise exceptions if they are called. Anyone making use of
    this file in their own projects does so at their own risk.

    Contributors:
    - Daniel S. Johnson (daniel.johnson-2@manchester.ac.uk)

    This class is an edited form of the DQ_HolonomicBase class, as that object assumes that the base is on the
    ground at all times. This limitation makes DQ_HolonomicBase unsuitable for legged locomotion, where the
    torso is free to move in the z-axis and can rotate about the x or y axes, hence a free floating base class
    was required. While this class was designed with legged robots in mind, it may also be useful for UAVs and
    aquatic ROVs.

    The configurations provided to the DQ_FreeFloatingBase are assumed to be of the form

        q = [ vec4(r)^T, vec3(t)^T ]^T,

    where r and t describe the orientation and translation of the base respectively. A version of this class
    that uses Euler angles (DQ_FreeFloatingBaseEuler) is also available, but its use is strongly discouraged
    unless you absolutely must use Euler angles, due to the well known inherent mathematical issues which
    arise from doing so.
*/

#include <dqrobotics/robot_modeling/DQ_FreeFloatingBase.h>

namespace DQ_robotics
{

    DQ_FreeFloatingBase::DQ_FreeFloatingBase()
    {
        dim_configuration_space_ = 7; // Changed from 3 in DQ_HolonomicBase
    }

    DQ DQ_FreeFloatingBase::raw_fkm(const VectorXd &q) const
    {
        // q is assumed to be structured as [ vec4(r)^T, vec3(t)^T ]^T
        DQ r(q.segment(0,4)),
           t(q.segment(4,3));

        DQ pose = r + E_*0.5*r*t;

        return pose;
    }

    DQ DQ_FreeFloatingBase::fkm(const VectorXd &q) const
    {
        return raw_fkm(q) * frame_displacement_;
    }

    DQ DQ_FreeFloatingBase::fkm(const VectorXd &q, const int &to_ith_link) const
    {
        if (to_ith_link != 6)
        {
            throw std::runtime_error(std::string("DQ_FreeFloatingBase(q,to_ith_link) only accepts to_ith_link=6"));
        }
        return fkm(q);
    }

    MatrixXd DQ_FreeFloatingBase::raw_pose_jacobian(const VectorXd &q, const int &to_link) const // Function needs reworking to reflect new situation
    {
        if (to_link >= 7 || to_link < 0)
        {
            throw std::runtime_error(std::string("Tried to access link index ") + std::to_string(to_link) + std::string(" which is unavailable."));
        }

        if (q.size() > 7)
        {
            throw std::runtime_error(std::string("DQ_FreeFloatingBase::raw_pose_jacobian was provided a vector with too many values (") + std::to_string(q.size()) + std::string(", should be 7)."));
        }

        DQ r(q.segment(0,4)),
           t(q.segment(4,3));

        VectorXd dual_by_r1 = vec4(0.5 * t);
        VectorXd dual_by_r2 = vec4(0.5 * i_ * t);
        VectorXd dual_by_r3 = vec4(0.5 * j_ * t);
        VectorXd dual_by_r4 = vec4(0.5 * k_ * t);

        VectorXd dual_by_t2 = vec4(0.5 * r * i_);
        VectorXd dual_by_t3 = vec4(0.5 * r * j_);
        VectorXd dual_by_t4 = vec4(0.5 * r * k_);

        MatrixXd pose_jacobian(8,7);

        pose_jacobian <<1, 0, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0, 0,
                        0, 0, 0, 1, 0, 0, 0,
                        dual_by_r1(0), dual_by_r2(0), dual_by_r3(0), dual_by_r4(0), dual_by_t2(0), dual_by_t3(0), dual_by_t4(0),
                        dual_by_r1(1), dual_by_r2(1), dual_by_r3(1), dual_by_r4(1), dual_by_t2(1), dual_by_t3(1), dual_by_t4(1),
                        dual_by_r1(2), dual_by_r2(2), dual_by_r3(2), dual_by_r4(2), dual_by_t2(2), dual_by_t3(2), dual_by_t4(2),
                        dual_by_r1(3); dual_by_r2(3), dual_by_r3(3), dual_by_r4(3), dual_by_t2(3), dual_by_t3(3), dual_by_t4(3);

        return pose_jacobian;
    }

    MatrixXd DQ_FreeFloatingBase::pose_jacobian(const VectorXd &q, const int &to_link) const
    {
        return haminus8(frame_displacement_) * raw_pose_jacobian(q, to_link);
    }

    MatrixXd DQ_FreeFloatingBase::pose_jacobian(const VectorXd &q) const
    {
        return pose_jacobian(q, get_dim_configuration_space() - 1);
    }

    // TODO: Docs need reworking to reflect new situation
    /**
     * @brief returns the Jacobian derivative (J_dot) that satisfies
              vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
              derivative of the pose and 'q_dot' represents the robot configuration velocities.
              This method does not take into account the base displacement.
     * @param q The VectorXd representing the robot configurations.
     * @param q_dot The VectorXd representing the robot configuration velocities.
     * @param to_link The ith link which we want to compute the Jacobian derivative.
     * @return a MatrixXd representing the desired Jacobian derivative.
     */
    MatrixXd DQ_FreeFloatingBase::raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_link) const
    // Function needs reworking to reflect new situation
    {
        throw std::runtime_error(std::string("DQ_FreeFloatingBase::raw_pose_jacobian_derivative has not been defined yet."));

        if (to_link >= 3 || to_link < 0)
        {
            throw std::runtime_error(std::string("Tried to access link index ") + std::to_string(to_link) + std::string(" which is unavailable."));
        }

        const double &x = q(0);
        const double &y = q(1);
        const double &phi = q(2);

        const double &x_dot = q_dot(0);
        const double &y_dot = q_dot(1);
        const double &phi_dot = q_dot(2);

        const double c = cos(phi / 2.0);
        const double s = sin(phi / 2.0);

        const double j71 = -0.25 * c * phi_dot;
        const double j62 = -j71;
        const double j13 = -j62;

        const double j72 = -0.25 * s * phi_dot;
        const double j61 = j72;
        const double j43 = j61;

        const double j63 = 0.25 * (-x_dot * s - 0.5 * x * c * phi_dot + y_dot * c - 0.5 * y * s * phi_dot);
        const double j73 = 0.25 * (-x_dot * c + 0.5 * x * s * phi_dot - y_dot * s - 0.5 * y * c * phi_dot);

        MatrixXd J_dot(8, 3);
        J_dot << 0.0, 0.0, j13,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, j43,
            0.0, 0.0, 0.0,
            j61, j62, j63,
            j71, j72, j73,
            0.0, 0.0, 0.0;
        return J_dot.block(0, 0, 8, to_link + 1);
    }

    // Docs need reworking to reflect new situation
    /**
     * @brief returns the Jacobian derivative 'J_dot' that satisfies
              vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
              derivative of the pose and 'q_dot' represents the robot configuration velocities.
     * @param q The VectorXd representing the robot configurations.
     * @param q_dot The VectorXd representing the robot configuration velocities.
     * @param to_ith_link The 'to_ith_link' link which we want to compute the Jacobian derivative.
     * @return a MatrixXd representing the desired Jacobian derivative.
     */
    MatrixXd DQ_FreeFloatingBase::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_link) const
    {
        return haminus8(frame_displacement_) * raw_pose_jacobian_derivative(q, q_dot, to_link);
    }

    /**
     * @brief returns the Jacobian derivative 'J_dot' that satisfies
              vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
              derivative of the pose and 'q_dot' represents the robot configuration velocities.
     * @param q The VectorXd representing the robot configurations.
     * @param q_dot The VectorXd representing the robot configuration velocities.
     * @return a MatrixXd representing the desired Jacobian derivative.
     */
    MatrixXd DQ_FreeFloatingBase::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
    {
        return pose_jacobian_derivative(q, q_dot, get_dim_configuration_space() - 1);
    }

}
