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

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_FREEFLOATINGBASE
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_FREEFLOATINGBASE

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_MobileBase.h>

namespace DQ_robotics
{


class DQ_FreeFloatingBase: public DQ_MobileBase
{
public:
    DQ_FreeFloatingBase();

    //Virtual method overloads (DQ_Kinematics)
    virtual DQ fkm(const VectorXd& q) const override;
    virtual DQ fkm(const VectorXd& q, const int& to_ith_link) const override;
    virtual MatrixXd pose_jacobian(const VectorXd& q, const int& to_link) const override;
    virtual MatrixXd pose_jacobian(const VectorXd& q) const override;
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                              const VectorXd& q_dot, const int& to_link) const override;
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                              const VectorXd& q_dot) const override;

    DQ raw_fkm(const VectorXd& q) const;
    MatrixXd raw_pose_jacobian(const VectorXd& q, const int& to_link=2) const;
    MatrixXd raw_pose_jacobian_derivative(const VectorXd& q,
                                          const VectorXd& q_dot, const int& to_link = 2) const;
};

}

#endif
