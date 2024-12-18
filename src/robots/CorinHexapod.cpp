/**
    This file is not (currently) part of the official release or development branches of DQ Robotics.
    This file incorporates elements of the official releases, but was written by Daniel S. Johnson
    without review by the original developers. As such, any problems introduced are my own responsibility.

    In accordance with the licencing arrangements for DQ Robotics itself, this file is distributed under
    an LGPL-3.0 license.

    As with all files in this fork, this file should be considered a work in progress. Anyone making use of
    this file in their own projects does so at their own risk.

    Contributors:
    - Daniel S. Johnson (daniel.johnson-2@manchester.ac.uk)

    This file defines a DQ_LeggedRobot object which models the Corin Hexapod, a bespoke platform designed
    and built at the University of Manchester by Wei Cheah.
*/

#include<dqrobotics/utils/DQ_Constants.h>
#include<dqrobotics/robots/CorinHexapod.h>

namespace DQ_robotics
{

DQ_LeggedRobot CorinHexapod::kinematics()
{
    // Create the torso
    DQ_LeggedRobot corin("standard","quaternion");

    // Create a matrix with the standard DH parameters for all legs
    Eigen::Matrix<double, 5, 3> leg_DH_matrix;
    leg_DH_matrix << 0     , 0   , 0,    //Theta
                     0     , 0   , 0,    //d
                     0.06  , 0.15, 0.134,//a
                     M_PI/2, 0   , 0,    //Alpha
                     0     , 0   , 0;    //Type of joint. 0= Revolute, 1= Prismatic

    // Create the leg objects. These are all identical, but have different base offsets.
    DQ_SerialManipulatorDH leg_1(leg_DH_matrix),
                           leg_2(leg_DH_matrix),
                           leg_3(leg_DH_matrix),
                           leg_4(leg_DH_matrix),
                           leg_5(leg_DH_matrix),
                           leg_6(leg_DH_matrix);

    // Define the leg offsets
    DQ leg_1_base_position(0,  0.115,  0.09, 0),
       leg_2_base_position(0,  0    ,  0.09, 0),
       leg_3_base_position(0, -0.115,  0.09, 0),
       leg_4_base_position(0,  0.115, -0.09, 0),
       leg_5_base_position(0,  0    , -0.09, 0),
       leg_6_base_position(0, -0.115, -0.09, 0);

    DQ leg_1_base_orientation=cos( 0.873/2)+k_*sin( 0.873/2),
       leg_2_base_orientation=cos( 1.571/2)+k_*sin( 1.571/2),
       leg_3_base_orientation=cos( 2.269/2)+k_*sin( 2.269/2),
       leg_4_base_orientation=cos(-0.873/2)+k_*sin(-0.873/2),
       leg_5_base_orientation=cos(-1.571/2)+k_*sin(-1.571/2),
       leg_6_base_orientation=cos(-2.269/2)+k_*sin(-2.269/2);

    DQ leg_1_offset=(leg_1_base_orientation+E_*0.5*leg_1_base_position*leg_1_base_orientation).normalize(),
       leg_2_offset=(leg_2_base_orientation+E_*0.5*leg_2_base_position*leg_2_base_orientation).normalize(),
       leg_3_offset=(leg_3_base_orientation+E_*0.5*leg_3_base_position*leg_3_base_orientation).normalize(),
       leg_4_offset=(leg_4_base_orientation+E_*0.5*leg_4_base_position*leg_4_base_orientation).normalize(),
       leg_5_offset=(leg_5_base_orientation+E_*0.5*leg_5_base_position*leg_5_base_orientation).normalize(),
       leg_6_offset=(leg_6_base_orientation+E_*0.5*leg_6_base_position*leg_6_base_orientation).normalize();

     // Generate a shared pointer to each leg
    std::shared_ptr<DQ_SerialManipulatorDH> leg_1_pointer = std::make_shared<DQ_SerialManipulatorDH>(leg_1),
                                            leg_2_pointer = std::make_shared<DQ_SerialManipulatorDH>(leg_2),
                                            leg_3_pointer = std::make_shared<DQ_SerialManipulatorDH>(leg_3),
                                            leg_4_pointer = std::make_shared<DQ_SerialManipulatorDH>(leg_4),
                                            leg_5_pointer = std::make_shared<DQ_SerialManipulatorDH>(leg_5),
                                            leg_6_pointer = std::make_shared<DQ_SerialManipulatorDH>(leg_6);

    // Add each leg to the robot
    corin.add_leg(leg_1_pointer, leg_1_offset);
    corin.add_leg(leg_2_pointer, leg_2_offset);
    corin.add_leg(leg_3_pointer, leg_3_offset);
    corin.add_leg(leg_4_pointer, leg_4_offset);
    corin.add_leg(leg_5_pointer, leg_5_offset);
    corin.add_leg(leg_6_pointer, leg_6_offset);

    // Return the result
    return corin;
}

}
