/**
(C) Copyright 2020-2022 DQ Robotics Developers

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
- Daniel S. Johnson - (daniel.johnson-2@manchester.ac.uk)
*/

#ifndef DQ_ROBOT_MODELLING_DQ_LEGGED_ROBOT_H
#define DQ_ROBOT_MODELLING_DQ_LEGGED_ROBOT_H

#include<vector>
#include<memory>
#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include<dqrobotics/robot_modeling/DQ_FreeFloatingBase.h>


namespace DQ_robotics
{

class DQ_LeggedRobot : public DQ_Kinematics
{
protected:
    std::vector<std::shared_ptr<DQ_Kinematics>> bodies_; // vector of robot bodies (0 is the torso, rest are legs)

    std::vector<DQ> base_frame_offsets_; // vector of poses representing the base frames of each body
    int number_of_legs_; // the number of legs attached, should be the length of the two above vectors

    void _check_to_ith_body(const int& to_ith_body) const;
    void _check_to_jth_link_of_ith_body(const int &to_jth_link, const int &of_ith_body) const;
    void _check_body_is_not_torso(const int &to_ith_body) const;

public:
    //Concrete methods
    DQ_LeggedRobot(const std::string type=std::string("standard")); // Constructor, automatically creates a DQ_FreeFloatingBase object to serve as the torso
    DQ_LeggedRobot()=delete; // Destructor

    DQ get_base_frame_of_ith_body(const int& ith_body) const;
    // void set_leg_base_frame(const int& ith_leg, const DQ& base_frame_pose);

    // std::vector<VectorXd> get_joint_configuration() const; 
    // void set_joint_configuration(const std::vector<VectorXd>& joint_configuration); 

    int get_number_bodies() const;
    int get_number_legs() const;

    void add_leg(std::shared_ptr<DQ_SerialManipulatorDH> leg, const DQ& base_frame_pose);

    DQ_SerialManipulatorDH get_leg_as_serial_manipulator_dh(const int& to_ith_leg) const;
    DQ_FreeFloatingBase get_torso() const;
    std::tuple<int,int> get_body_and_link_from_dof_index(const int &to_ith_DOF) const;

    std::vector<DQ> fkm_all_feet(const VectorXd& q) const;
    DQ fkm_by_body(const VectorXd& q, const int& to_ith_body) const;
    DQ fkm_by_body(const VectorXd& q, const int& to_ith_body, const int& to_jth_dof) const;
    DQ raw_fkm_by_body(const VectorXd& q, const int& to_ith_body, const int& to_jth_dof) const;

    std::vector<MatrixXd> pose_jacobian_all_feet(const VectorXd& q) const;
    MatrixXd pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body) const;
    MatrixXd pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const;
    MatrixXd raw_pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const;

    MatrixXd raw_pose_jacobian_derivative_by_body(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_chain, const int &to_jth_link) const;

    // Implementations of abstract methods inherited from DQ_Kinematics
    DQ fkm(const VectorXd& q) const override;
    DQ fkm(const VectorXd&, const int& to_ith_link) const override;
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_ith_link) const override;
    MatrixXd pose_jacobian(const VectorXd& q) const override;
    MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                      const VectorXd& q_dot,
                                      const int& to_ith_link) const override; //To be implemented.
    MatrixXd pose_jacobian_derivative (const VectorXd& q,
                                       const VectorXd& q_dot) const override; //To be implemented.
};

}
#endif