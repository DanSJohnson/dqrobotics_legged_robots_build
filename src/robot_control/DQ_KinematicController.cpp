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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include <dqrobotics/robot_control/DQ_KinematicController.h>
#include <stdexcept>

namespace DQ_robotics
{

DQ_KinematicController::DQ_KinematicController(DQ_Kinematics* robot)
{
    robot_             = robot;
    control_objective_ = ControlObjective::None;
    gain_              = MatrixXd::Zero(1,1);

    is_stable_           = false;
    last_error_signal_   = VectorXd::Zero(1);
    last_error_signal_   = VectorXd::Zero(1);
    stability_threshold_ = 0.0;
}

ControlObjective DQ_KinematicController::get_control_objective() const
{
    return control_objective_;
}

MatrixXd DQ_KinematicController::get_jacobian(const VectorXd &q, const DQ &primitive) const
{
    const MatrixXd J_pose = robot_->pose_jacobian(q,robot_->get_dim_configuration_space());
    const DQ       x_pose = robot_->fkm(q);

    switch(control_objective_)
    {
    case ControlObjective::None:
        throw std::runtime_error("The control objective must be initialized with set_control_objective()");

    case ControlObjective::Distance:
        return DQ_Kinematics::distance_jacobian(J_pose,x_pose);

    case ControlObjective::Line:
        return DQ_Kinematics::line_jacobian(J_pose,x_pose,primitive);

    case ControlObjective::Plane:
        return DQ_Kinematics::plane_jacobian(J_pose,x_pose,primitive);

    case ControlObjective::Rotation:
        return DQ_Kinematics::rotation_jacobian(J_pose);

    case ControlObjective::Translation:
        return DQ_Kinematics::translation_jacobian(J_pose,x_pose);

    case ControlObjective::Pose:
        return J_pose;
    }
}

VectorXd DQ_KinematicController::get_task_variable(const VectorXd &q, const DQ &primitive) const
{
    const DQ       x_pose = robot_->fkm(q);

    switch(control_objective_)
    {
    case ControlObjective::None:
        throw std::runtime_error("The control objective must be initialized with set_control_objective()");

    case ControlObjective::Distance:
    {
        VectorXd p = vec4(translation(x_pose));
        return p.transpose()*p;
    }

    case ControlObjective::Line:
        return vec8(Ad(x_pose,primitive));

    case ControlObjective::Plane:
        return vec4(Adsharp(x_pose,primitive));

    case ControlObjective::Rotation:
        return vec4(rotation(x_pose));

    case ControlObjective::Translation:
        return vec4(translation(x_pose));

    case ControlObjective::Pose:
        return vec8(x_pose);
    }
}

bool DQ_KinematicController::is_set() const
{
    if(control_objective_==ControlObjective::None)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool DQ_KinematicController::is_stable() const
{
    return is_stable_;
}

void DQ_KinematicController::set_control_objective(const ControlObjective &control_objective)
{
    control_objective_ = control_objective;

    switch(control_objective)
    {
    case ControlObjective::Distance:
        last_error_signal_ = VectorXd::Zero(1);
        break;
    case ControlObjective::Line: //This was intentional https://en.cppreference.com/w/cpp/language/attributes/fallthrough
    case ControlObjective::Plane: //This was intentional https://en.cppreference.com/w/cpp/language/attributes/fallthrough
    case ControlObjective::Pose:
        last_error_signal_ = VectorXd::Zero(8);
        break;
    case ControlObjective::Rotation:
    case ControlObjective::Translation:
        last_error_signal_ = VectorXd::Zero(4);
        break;
    case ControlObjective::None:
        break;
    }

}

void DQ_KinematicController::set_gain(const MatrixXd &gain)
{
    gain_ = gain;
}

void DQ_KinematicController::set_stability_threshold(const double &threshold)
{
    stability_threshold_ = threshold;
}

}