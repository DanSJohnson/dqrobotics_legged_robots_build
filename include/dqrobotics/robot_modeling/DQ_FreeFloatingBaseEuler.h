/**
    This file is not (currently) part of the official release or development branches of DQ Robotics.
    This file incorporates elements of the official releases, but was written by Daniel S. Johnson
    without review by the original developers. As such, any problems introduced are my own responsibility.

    In accordance with the licencing arrangements for DQ Robotics itself, this file is distributed under
    an LGPL-3.0 license.

    As with all files in this fork, this file should be considered a work in progress. Anyone making use
    of this file in their own projects does so at their own risk.

    Contributors:
    - Daniel S. Johnson (daniel.johnson-2@manchester.ac.uk)

    This class is similar to the DQ_FreeFloatingBaseEuler class, except that the configuration vector is assumed
    to take the following form:

        q = [r_x, r_y, r_z, t_x, t_y, t_z]^T,

    where r_x, r_y and r_z are the Euler angles of the base, applied following a ZYX extrinsic convention (angles
    are given with respect to the world frame axes in radians). The elements t_x, t_y and t_z denote the position
    of the base frame with respect to the world frame.

    In almost all cases it would be strongly advisable to use the standard DQ_FreeFloatingBase class rather than
    this one due to the well documented problems that arise from using Euler angles to represent rotations. As
    such, this class is not used by default in any of the other classes that come with this fork. However, in some cases
    it may be necessary to interface with Euler angles (e.g., if other software in the stack uses them) and so
    this class is made available at users own risk. The DQ_FreeFloatingBaseEuler can be used as the torso
    in a DQ_LeggedRobot object by specifying base = "Euler" in the DQ_LeggedRobot constructor.
*/

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_FREEFLOATINGBASEEULER
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_FREEFLOATINGBASEEULER

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_MobileBase.h>

namespace DQ_robotics
{


class DQ_FreeFloatingBaseEuler: public DQ_MobileBase
{
public:
    DQ_FreeFloatingBaseEuler();

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
