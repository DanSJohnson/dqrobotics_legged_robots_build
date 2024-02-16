/**
(C) Copyright 2019 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Daniel S. Johnson (daniel.johnson-2@manchester.ac.uk)
*/

/* 
   This file is edited from DQ_HolonomicBase, as that object assumes that the base is on the ground at all times.
   This is not suitable for legged locomotion, where the torso is free to move in the z-axis and can rotate about
   the x or y axes. 
*/

#include<dqrobotics/robot_modeling/DQ_FreeFloatingBase.h>

namespace DQ_robotics
{

DQ_FreeFloatingBase::DQ_FreeFloatingBase()
{
    dim_configuration_space_ = 6; //Changed from 3 in DQ_HolonomicBase
}

DQ DQ_FreeFloatingBase::raw_fkm(const VectorXd& q) const //Reworked to take x,y,z coordinates and rpy euler angles.
{
    const double& x     = q(0);
    const double& y     = q(1);
    const double& z     = q(2);
    const double& roll  = q(3);
    const double& pitch = q(4);
    const double& yaw   = q(5);

    DQ rx, ry, rz, r, t, pose;
    rx = cos(roll/2) + i_*sin(roll/2);
    ry = cos(pitch/2) + j_*sin(pitch/2);
    rz = cos(yaw/2) + k_*sin(yaw/2);

    r= rx*ry*rz;

    t = x*i_ + y*j_ + z*k_;

    pose = r + E_*0.5*t*r;
    pose=pose.normalize();

    return pose;
}

DQ DQ_FreeFloatingBase::fkm(const VectorXd& q) const
{
    return raw_fkm(q)*frame_displacement_;
}

DQ DQ_FreeFloatingBase::fkm(const VectorXd &q, const int& to_ith_link) const
{
    if(to_ith_link != 5)
    {
        throw std::runtime_error(std::string("DQ_FreeFloatingBase(q,to_ith_link) only accepts to_ith_link=5"));
    }
    return fkm(q);
}

MatrixXd DQ_FreeFloatingBase::raw_pose_jacobian(const VectorXd& q, const int& to_link) const // Function needs reworking to reflect new situation
{
    if(to_link >= 6 || to_link < 0)
    {
        throw std::runtime_error(std::string("Tried to access link index ") + std::to_string(to_link) + std::string(" which is unavailable."));
    }

    if(q.size() > 6){
        throw std::runtime_error(std::string("DQ_FreeFloatingBase::raw_pose_jacobian was provided a vector with too many values (") + std::to_string(q.size()) + std::string(", should be 6). Possibly a pose was provided instead by mistake?"));
    }

    // Extract values from configuration velocity
    const double& x     = q(0);
    const double& y     = q(1);
    const double& z     = q(2);
    const double& roll  = q(3);
    const double& pitch = q(4);
    const double& yaw   = q(5);

    // Shorthands for sine and cosine of angles
    const double cr = cos(roll/2.0);
    const double sr = sin(roll/2.0);
    const double cp = cos(pitch/2.0);
    const double sp = sin(pitch/2.0);
    const double cy = cos(yaw/2.0);
    const double sy = sin(yaw/2.0);

    // rotation DQs
    DQ rx = cr+i_*sr;
    DQ ry = cp+j_*sp;
    DQ rz = cy+k_*sy;
    DQ r=rx*ry*rz;

    // derivatives of rotation DQs by their respective euler angles
    DQ d_rx_by_d_roll=-0.5*sr+0.5*i_*cr;
    DQ d_ry_by_d_pitch=-0.5*sp+0.5*j_*cp;
    DQ d_rz_by_d_yaw=-0.5*sy+0.5*k_*cy;

    // derivatives of primary part of the pose by each euler angle
    DQ d_r_by_d_roll=d_rx_by_d_roll*ry*rz;
    DQ d_r_by_d_pitch=rx*d_ry_by_d_pitch*rz;
    DQ d_r_by_d_yaw=rx*ry*d_rz_by_d_yaw;

    // vector forms of the above
    Eigen::VectorXd vec_d_r_by_d_roll=vec4(d_r_by_d_roll);
    Eigen::VectorXd vec_d_r_by_d_pitch=vec4(d_r_by_d_pitch);
    Eigen::VectorXd vec_d_r_by_d_yaw=vec4(d_r_by_d_yaw);

    // derivatives of the position DQ t by each coordinate
    DQ t=x*i_+y*j_+z*k_;
    DQ d_t_by_d_x=i_;
    DQ d_t_by_d_y=j_;
    DQ d_t_by_d_z=k_;

    // derivatives of the dual part of the pose by each coordinate
    DQ d_dual_by_d_x=0.5*d_t_by_d_x*r;
    DQ d_dual_by_d_y=0.5*d_t_by_d_y*r;
    DQ d_dual_by_d_z=0.5*d_t_by_d_z*r;

    // vector forms of the above
    Eigen::VectorXd vec_d_dual_by_d_x=vec4(d_dual_by_d_x);
    Eigen::VectorXd vec_d_dual_by_d_y=vec4(d_dual_by_d_y);
    Eigen::VectorXd vec_d_dual_by_d_z=vec4(d_dual_by_d_z);

    // derivatives of the dual part of the pose by each euler angle
    DQ d_dual_by_d_roll=0.5*t*d_r_by_d_roll;
    DQ d_dual_by_d_pitch=0.5*t*d_r_by_d_pitch;
    DQ d_dual_by_d_yaw=0.5*t*d_r_by_d_yaw;

    // vector forms of the above
    Eigen::VectorXd vec_d_dual_by_d_roll=vec4(d_dual_by_d_roll);
    Eigen::VectorXd vec_d_dual_by_d_pitch=vec4(d_dual_by_d_pitch);
    Eigen::VectorXd vec_d_dual_by_d_yaw=vec4(d_dual_by_d_yaw);

    // Combine all the above into the Jacobian
    MatrixXd J(8,6);
    J << 0, 0, 0, vec_d_r_by_d_roll(0), vec_d_r_by_d_pitch(0), vec_d_r_by_d_yaw(0),
         0, 0, 0, vec_d_r_by_d_roll(1), vec_d_r_by_d_pitch(1), vec_d_r_by_d_yaw(1),
         0, 0, 0, vec_d_r_by_d_roll(2), vec_d_r_by_d_pitch(2), vec_d_r_by_d_yaw(2),
         0, 0, 0, vec_d_r_by_d_roll(3), vec_d_r_by_d_pitch(3), vec_d_r_by_d_yaw(3),
         vec_d_dual_by_d_x(0), vec_d_dual_by_d_y(0), vec_d_dual_by_d_z(0), vec_d_dual_by_d_roll(0), vec_d_dual_by_d_pitch(0), vec_d_dual_by_d_yaw(0),
         vec_d_dual_by_d_x(1), vec_d_dual_by_d_y(1), vec_d_dual_by_d_z(1), vec_d_dual_by_d_roll(1), vec_d_dual_by_d_pitch(1), vec_d_dual_by_d_yaw(1),
         vec_d_dual_by_d_x(2), vec_d_dual_by_d_y(2), vec_d_dual_by_d_z(2), vec_d_dual_by_d_roll(2), vec_d_dual_by_d_pitch(2), vec_d_dual_by_d_yaw(2),
         vec_d_dual_by_d_x(3), vec_d_dual_by_d_y(3), vec_d_dual_by_d_z(3), vec_d_dual_by_d_roll(3), vec_d_dual_by_d_pitch(3), vec_d_dual_by_d_yaw(3);
    
    return J.block(0,0,8,to_link+1);
}

MatrixXd DQ_FreeFloatingBase::pose_jacobian(const VectorXd &q, const int &to_link) const
{
    return haminus8(frame_displacement_)*raw_pose_jacobian(q,to_link);
}

MatrixXd DQ_FreeFloatingBase::pose_jacobian(const VectorXd &q) const
{
    return pose_jacobian(q,get_dim_configuration_space()-1);
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
    
    if(to_link >= 3 || to_link < 0)
    {
        throw std::runtime_error(std::string("Tried to access link index ") + std::to_string(to_link) + std::string(" which is unavailable."));
    }

    const double& x   = q(0);
    const double& y   = q(1);
    const double& phi = q(2);

    const double& x_dot   = q_dot(0);
    const double& y_dot   = q_dot(1);
    const double& phi_dot = q_dot(2);

    const double c = cos(phi/2.0);
    const double s = sin(phi/2.0);

    const double j71 = -0.25*c*phi_dot;
    const double j62 = -j71;
    const double j13 = -j62;

    const double j72 = -0.25*s*phi_dot;
    const double j61 = j72;
    const double j43 = j61;

    const double j63 = 0.25 * (-x_dot*s - 0.5*x*c*phi_dot + y_dot*c - 0.5*y*s*phi_dot);
    const double j73 = 0.25 * (-x_dot*c + 0.5*x*s*phi_dot - y_dot*s - 0.5*y*c*phi_dot);

    MatrixXd J_dot(8,3);
    J_dot <<    0.0, 0.0, j13,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, j43,
            0.0, 0.0, 0.0,
            j61, j62, j63,
            j71, j72, j73,
            0.0, 0.0, 0.0;
    return J_dot.block(0,0,8,to_link+1);
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
    return haminus8(frame_displacement_)*raw_pose_jacobian_derivative(q, q_dot, to_link);
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
    return pose_jacobian_derivative(q, q_dot, get_dim_configuration_space()-1);
}

}
