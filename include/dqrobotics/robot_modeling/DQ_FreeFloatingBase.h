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
