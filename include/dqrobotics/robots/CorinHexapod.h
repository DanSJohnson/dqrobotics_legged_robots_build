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

#ifndef DQ_ROBOTS_CORINHEXAPOD_H
#define DQ_ROBOTS_CORINHEXAPOD_H

#include<dqrobotics/robot_modeling/DQ_LeggedRobot.h>

namespace DQ_robotics
{

class CorinHexapod
{
public:
    static DQ_LeggedRobot kinematics();
};

}

#endif
