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
- Daniel S. Johnson (daniel.johnson-2@manchester.ac.uk)
*/

#include<dqrobotics/robot_modeling/DQ_LeggedRobot.h>
#include<vector>

namespace DQ_robotics
{

void DQ_LeggedRobot::_check_to_ith_body(const int &to_ith_body) const
{
    if(to_ith_body > number_of_legs_ || to_ith_body < 0)
    {
        throw std::runtime_error(std::string("Tried to access body index ") + std::to_string(to_ith_body) + std::string(" which is unavailable."));
    }
}

void DQ_LeggedRobot::_check_body_is_not_torso(const int &to_ith_body) const
{
    if(to_ith_body == 0)
    {
        throw std::runtime_error(std::string("Tried to access body index 0, which corresponds to the torso, in a function which is only defined for legs (indicies > 0)"));
    }
}

void DQ_LeggedRobot::_check_to_jth_link_of_ith_body(const int &to_jth_link, const int &of_ith_body) const
{
    _check_to_ith_body(of_ith_body);
    if( (to_jth_link >= static_cast<int>(bodies_[of_ith_body]->get_dim_configuration_space())) || to_jth_link < 0)
    {
        throw std::runtime_error(
                    std::string("Tried to access link/DOF index ")
                    + std::to_string(to_jth_link)
                    + std::string(" of body with index ")
                    + std::to_string(of_ith_body)
                    + std::string(" which is unavailable.")
                    );
    }
}

DQ_LeggedRobot::DQ_LeggedRobot(const std::string type)
{
    DQ_FreeFloatingBase torso;
    dim_configuration_space_=6;
    number_of_legs_=0;
    bodies_.push_back(std::make_shared<DQ_FreeFloatingBase>(torso));
    DQ torso_offset(1);
    base_frame_offsets_.push_back(torso_offset);

    if(type == std::string("standard"))
    {
        //Nothing to do
    }
    else if(type == std::string("reversed"))
    {
        throw std::runtime_error(std::string("Reversed type DQ_LeggedRobot is not implemented yet"));
    }
    else
    {
        throw std::runtime_error(std::string("Invalid type: ") + type);
    }
}

DQ DQ_LeggedRobot::get_base_frame_of_ith_body(const int& ith_body) const
{
    _check_to_ith_body(ith_body);
    return base_frame_offsets_[ith_body];
}

int DQ_LeggedRobot::get_number_bodies() const
{
    return number_of_legs_+1;    
}

int DQ_LeggedRobot::get_number_legs() const
{
    return number_of_legs_;    
}

DQ_SerialManipulatorDH DQ_LeggedRobot::get_leg_as_serial_manipulator_dh(const int &to_ith_body) const
{
    _check_to_ith_body(to_ith_body);
    _check_body_is_not_torso(to_ith_body);

    try
    {
        return DQ_SerialManipulatorDH(*dynamic_cast<DQ_SerialManipulatorDH*>(bodies_[static_cast<std::vector<DQ_Kinematics*>::size_type>(to_ith_body)].get()));
    } catch (const std::bad_cast& e)
    {
        throw std::runtime_error("Index requested in get_leg_as_serial_manipulator_dh is not a SerialManipulator" + std::string(e.what()));
    }
}

DQ_FreeFloatingBase DQ_LeggedRobot::get_torso() const
{
    return DQ_FreeFloatingBase(*dynamic_cast<DQ_FreeFloatingBase*>(bodies_[static_cast<std::vector<DQ_Kinematics*>::size_type>(0)].get()));
}

void DQ_LeggedRobot::add_leg(std::shared_ptr<DQ_SerialManipulatorDH> leg, const DQ& base_frame_pose)
{
    dim_configuration_space_+= leg->get_dim_configuration_space();
    bodies_.push_back(leg);
    base_frame_offsets_.push_back(base_frame_pose);
    number_of_legs_+=1;
}

DQ DQ_LeggedRobot::fkm(const VectorXd& q) const
{
    throw std::runtime_error("DQ fkm(const VectorXd& q) is not defined for DQ_LeggedRobot because there are multiple end effectors. Please use std::vector<DQ> fkm_all_feet(const VectorXd& q) instead if you want the poses of all end effectors, or specify an end effector using DQ fkm_by_body(const VectorXd& q, const int& to_ith_body) if you want a specific end effector.");
    // return reference_frame_ * raw_fkm(q);
}

DQ DQ_LeggedRobot::fkm(const VectorXd& q, const int& to_ith_link) const
{
    throw std::runtime_error("DQ fkm(const VectorXd& q, const int& to_ith_link) is not defined for DQ_LeggedRobot because there are multiple end effectors. Please specify a specific end effector using DQ fkm_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof).");
    // return reference_frame_ * raw_fkm(q, to_ith_link);
}

std::tuple<int,int> DQ_LeggedRobot::get_body_and_link_from_dof_index(const int &to_ith_DOF) const
{
    int ith_body = 0;
    int jth_dof = 0;
    int n = to_ith_DOF;

    for(size_t ith=0;ith<number_of_legs_+1;ith++)
    {
        if( (n - bodies_[ith]->get_dim_configuration_space()) >= 0)
        {
            ith_body++;
            n = n - bodies_[ith]->get_dim_configuration_space();
        }
        else
        {
            jth_dof = n;
            return std::make_tuple(ith_body, jth_dof);
        }
    }
    throw std::runtime_error("Unable to get body and link from dof index "+std::to_string(to_ith_DOF));
}

DQ DQ_LeggedRobot::fkm_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const
{
    return reference_frame_ * raw_fkm_by_body(q, to_ith_body, to_jth_dof);
}

DQ DQ_LeggedRobot::fkm_by_body(const VectorXd &q, const int &to_ith_body) const
{
    return reference_frame_ * raw_fkm_by_body(q, to_ith_body, bodies_[to_ith_body]->get_dim_configuration_space()-1);
}

std::vector<DQ> DQ_LeggedRobot::fkm_all_feet(const VectorXd& q) const
{
    std::vector<DQ> poses;
    for(int i=1;i<number_of_legs_+1; i++){
        poses.push_back(reference_frame_ * raw_fkm_by_body(q, i, bodies_[i]->get_dim_configuration_space()-1));
    }

    return poses;
}

// DQ DQ_LeggedRobot::raw_fkm(const VectorXd &q) const
// {// This is the instance of raw_fkm called if we want the fkm for every foot
//     return raw_fkm_by_body(q, number_of_legs_, bodies_[bodies_.size()-1]->get_dim_configuration_space()-1);
// }

// DQ DQ_LeggedRobot::raw_fkm(const VectorXd &q, const int &to_ith_link) const
// {
//     int to_ith_chain;
//     int to_jth_link;
//     std::tie(to_ith_chain,to_jth_link) = get_chain_and_link_from_index(to_ith_link);

//     return raw_fkm_by_chain(q, to_ith_chain, to_jth_link);
// }

DQ DQ_LeggedRobot::raw_fkm_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const
{
    _check_q_vec(q);
    _check_to_ith_body(to_ith_body);

    DQ pose(1);

    int q_counter = 0;
    int current_robot_dim;
    int skipped_dofs=6;
    VectorXd current_robot_q;
 
    // Always call the torso's fkm function
    current_robot_q    = q.segment(0,6);
    pose = pose * bodies_[0]->fkm(current_robot_q);
    
    //If its a leg, then also handle the leg in question
    if(to_ith_body>0)
    {
        current_robot_dim  = bodies_[to_ith_body]->get_dim_configuration_space();

        for(int link_num=1; link_num<to_ith_body; link_num++){
            skipped_dofs+=bodies_[link_num]->get_dim_configuration_space();
        }

        current_robot_q    = q.segment(skipped_dofs,current_robot_dim);
        pose = pose * base_frame_offsets_[to_ith_body] * bodies_[to_ith_body]->fkm(current_robot_q,to_jth_dof);
    }

    return pose;
}

// DQ DQ_LeggedRobot::raw_fkm_by_chain(const VectorXd &q, const int &to_ith_chain) const
// {
//     _check_q_vec(q);
//     _check_to_ith_chain(to_ith_chain);

//     return raw_fkm_by_chain(q,to_ith_chain,chain_[to_ith_chain]->get_dim_configuration_space()-1);
// }

MatrixXd DQ_LeggedRobot::raw_pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const
{
    _check_q_vec(q);
    _check_to_jth_link_of_ith_body(to_jth_dof, to_ith_body);

    DQ x_0_to_n = raw_fkm_by_body(q,to_ith_body,to_jth_dof);

    //Implementation based on that in DQ_SerialWholeBodyDH, which was based on MATLAB implementation
    MatrixXd J_pose(8,6);
    if(to_ith_body>0){
        J_pose.resize(8,6+bodies_[to_ith_body]->get_dim_configuration_space());
    }
    
    // Always do the torso
    const DQ x_0_to_iplus1 = bodies_[0]->fkm(q.segment(0,6));
    const DQ x_iplus1_to_n = conj(x_0_to_iplus1)*x_0_to_n;
    MatrixXd torso_J=haminus8(x_iplus1_to_n)*bodies_[0]->pose_jacobian(q.segment(0,6));
    for(int j=0;j<6;j++)
    {
        J_pose.col(j)=torso_J.col(j);
    }
    
    // If its a leg, add the leg part on
    if(to_ith_body>0){  

        int dim = bodies_[to_ith_body]->get_dim_configuration_space();

        const DQ x_0_to_iplus1 = raw_fkm_by_body(q, to_ith_body, bodies_[to_ith_body]->get_dim_configuration_space()-1);
        const DQ x_iplus1_to_n = conj(x_0_to_iplus1)*x_0_to_n;

        int skipped_dofs=6;
        for(int link_num=1; link_num<to_ith_body; link_num++){
            skipped_dofs+=bodies_[link_num]->get_dim_configuration_space();
        }

        const VectorXd q_iplus1 = q.segment(skipped_dofs,dim);

        MatrixXd leg_J=hamiplus8(raw_fkm_by_body(q,0,5))*haminus8(x_iplus1_to_n)*bodies_[to_ith_body]->pose_jacobian(q_iplus1,to_jth_dof);

        for(int j=0;j<dim;j++)
        {
            J_pose.col(6+j)=leg_J.col(j);
        }
    }
    return J_pose;
}

MatrixXd DQ_LeggedRobot::pose_jacobian(const VectorXd &q, const int &to_ith_link) const
{
    // int to_ith_chain;
    // int to_jth_link;
    // std::tie(to_ith_chain,to_jth_link) = get_chain_and_link_from_index(to_ith_link);
    // return raw_pose_jacobian_by_chain(q,to_ith_chain,to_jth_link);

    throw std::runtime_error("MatrixXd pose_jacobian(const VectorXd& q, const int& to_ith_link) is not defined for DQ_LeggedRobot because there are multiple end effectors. Please specify a specific end effector using DQ pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof).");

}

MatrixXd DQ_LeggedRobot::pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const
{
    // int to_ith_chain;
    // int to_jth_link;
    // std::tie(to_ith_chain,to_jth_link) = get_chain_and_link_from_index(to_ith_link);
    return raw_pose_jacobian_by_body(q,to_ith_body,to_jth_dof);

}

MatrixXd DQ_LeggedRobot::pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body) const
{
    // int to_ith_chain;
    // int to_jth_link;
    // std::tie(to_ith_chain,to_jth_link) = get_chain_and_link_from_index(to_ith_link);
    return raw_pose_jacobian_by_body(q,to_ith_body,bodies_[to_ith_body]->get_dim_configuration_space()-1);

}

MatrixXd DQ_LeggedRobot::pose_jacobian(const VectorXd &q) const
{
    // return pose_jacobian(q, get_dim_configuration_space()-1);
    throw std::runtime_error("MatrixXd pose_jacobian(const VectorXd& q) is not defined for DQ_LeggedRobot because there are multiple end effectors. Please use std::vector<MatrixXd> pose_jacobian_all_feet(const VectorXd& q) instead if you want the pose Jacobians of all end effectors, or specify an end effector using MatrixXd pose_jacobian_by_body(const VectorXd& q, const int& to_ith_body) if you want a specific end effector.");

}

std::vector<MatrixXd> DQ_LeggedRobot::pose_jacobian_all_feet(const VectorXd& q) const
{
    std::vector<MatrixXd> Jacobians;
    for(int i=1;i<number_of_legs_+1; i++){
        Jacobians.push_back(pose_jacobian_by_body(q, i, bodies_[i]->get_dim_configuration_space()-1));
    }
    return Jacobians;
}

//To be implemented.
MatrixXd DQ_LeggedRobot::raw_pose_jacobian_derivative_by_body(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_chain, const int &to_jth_link) const
{
    throw std::runtime_error("pose_jacobian_derivative_by_body is not implemented yet.");
}

MatrixXd DQ_LeggedRobot::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    int to_ith_chain;
    int to_jth_link;

    std::tie(to_ith_chain,to_jth_link) = get_body_and_link_from_dof_index(to_ith_link);
    return raw_pose_jacobian_derivative_by_body(q, q_dot, to_ith_chain,to_ith_link);
}

MatrixXd DQ_LeggedRobot::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    return pose_jacobian_derivative(q,q_dot,  get_dim_configuration_space()-1);
}

// void DQ_LeggedRobot::set_effector(const DQ &effector)
// {
//     try
//     {
//         std::dynamic_pointer_cast<DQ_SerialManipulator>(chain_[chain_.size()-1])->set_effector(effector);
//     }
//     catch (const std::bad_cast& e)
//     {
//         throw std::runtime_error("The last element of the chain needs to be a DQ_SerialManipulator to use set_effector in a DQ_WholeBody " + std::string(e.what()));
//     }
// }

}
