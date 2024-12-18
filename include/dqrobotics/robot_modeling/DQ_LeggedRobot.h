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

    This class is an edited form of the DQ_SerialWholeBody class. Some functions in this file are based on similar
    functions in a private fork of the DQ Robotics Matlab Distribution by Frederico Fernandes Afonso Silva.

    The configurations provided to the DQ_FreeFloatingBase class is assumed to be of the form

    q = [ base_configuration^T, leg_1_configuration^T, ... leg_n_configuration^T ]^T.

    How the base_configuration is formatted depends on which base is used, either DQ_FreeFloatingBase (the default) or
    DQ_FreeFloatingBaseEuler. See the files for these classes for details on their configurations. All the legs are
    described by DQ_SerialManipulatorDH objects.

    An example of how to create a DQ_LeggedRobot object is given in the file src/robots/CorinHexapod.cpp
*/

#ifndef DQ_ROBOT_MODELLING_DQ_LEGGED_ROBOT_H
#define DQ_ROBOT_MODELLING_DQ_LEGGED_ROBOT_H

#include <vector>
#include <memory>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_FreeFloatingBase.h>
#include <dqrobotics/robot_modeling/DQ_FreeFloatingBaseEuler.h>

namespace DQ_robotics
{

    class DQ_LeggedRobot : public DQ_Kinematics
    {
    protected:
        std::vector<std::shared_ptr<DQ_Kinematics>> bodies_; // vector of robot bodies (0 is the torso, rest are legs)

        std::vector<DQ> base_frame_offsets_; // vector of poses representing the base frames of each body
        int number_of_legs_;                 // the number of legs attached, should be the length of the two above vectors

        void _check_to_ith_body(const int &to_ith_body) const;
        void _check_to_jth_link_of_ith_body(const int &to_jth_link, const int &of_ith_body) const;
        void _check_body_is_not_torso(const int &to_ith_body) const;

    public:
        // Concrete methods
        DQ_LeggedRobot(const std::string type = std::string("standard"), const std::string base = std::string("quaternion")); // Constructor.
        // The constructor automatically a base to serve as the robot torso. If base is set to "quaternion" (the default) then
        // the base will be a DQ_FreeFloatingBase object. If it is instead set to "Euler" then it will be a DQ_FreeFloatingBaseEuler
        // object. The use of the Euler Angles base is not recommended.
        DQ_LeggedRobot() = delete;                                        // Destructor

        DQ get_base_frame_of_ith_body(const int &ith_body) const;
        // void set_leg_base_frame(const int& ith_leg, const DQ& base_frame_pose);

        // std::vector<VectorXd> get_joint_configuration() const;
        // void set_joint_configuration(const std::vector<VectorXd>& joint_configuration);

        int get_number_bodies() const;
        int get_number_legs() const;
        int get_number_of_links_across_all_bodies() const;

        void add_leg(std::shared_ptr<DQ_SerialManipulatorDH> leg, const DQ &base_frame_pose);

        DQ_SerialManipulatorDH get_leg_as_serial_manipulator_dh(const int &to_ith_leg) const;
        DQ_FreeFloatingBase get_torso() const;
        std::tuple<int, int> get_body_and_link_from_dof_index(const int &to_ith_DOF) const;

        std::vector<DQ> fkm_all_feet(const VectorXd &q) const;
        DQ fkm_by_body(const VectorXd &q, const int &to_ith_body) const;
        DQ fkm_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const;
        DQ raw_fkm_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const;
        VectorXd foot_position_as_vector(const VectorXd &q, const int &to_ith_body) const;

        std::vector<MatrixXd> pose_jacobian_all_feet(const VectorXd &q) const;
        MatrixXd pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body) const;
        MatrixXd pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const;
        MatrixXd raw_pose_jacobian_by_body(const VectorXd &q, const int &to_ith_body, const int &to_jth_dof) const;

        MatrixXd raw_pose_jacobian_derivative_by_body(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_chain, const int &to_jth_link) const;

        // Implementations of abstract methods inherited from DQ_Kinematics
        DQ fkm(const VectorXd &q) const override;
        DQ fkm(const VectorXd &, const int &to_ith_link) const override;
        MatrixXd pose_jacobian(const VectorXd &q, const int &to_ith_link) const override;
        MatrixXd pose_jacobian(const VectorXd &q) const override;
        MatrixXd pose_jacobian_derivative(const VectorXd &q,
                                          const VectorXd &q_dot,
                                          const int &to_ith_link) const override; // To be implemented.
        MatrixXd pose_jacobian_derivative(const VectorXd &q,
                                          const VectorXd &q_dot) const override; // To be implemented.
    };

}
#endif