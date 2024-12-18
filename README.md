# DQ Robotics Legged Robots Build
This repo is a fork of the [original DQ Robotics library in C++](https://github.com/dqrobotics/cpp) which has been extended to include classes for legged robots. 

The repo is currently a work in progress, and anyone making use of this code does so at their own risk.

This repo is the project of one PhD student for my personal use. As such, any additions to the base DQ Robotics library made in this repo have not been subject to the usual rigorous review process typically applied to DQ Robotics code. If there are issues in the new classes, then I take full responsibility for them.

The docs for DQ Robotics can be found [here](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/cpp.html). Installation and building of this fork should be similar to the process for DQ_Robotics_cpp, except that the source should be cloned from this repo instead.

## What’s New

A lot of very powerful functionality is already present in DQ Robotics and for information on that I refer you to the [website for the original library](https://dqrobotics.github.io/). Here I will only offer brief notes on my additions.

There are three new classes in this fork to enable the modelling of new kinds of robots. These are:
- DQ_FreeFloatingBase - A class to represent free floating robot bases such as those of legged robots.
- DQ_FreeFloatingBaseEuler - A version of DQ_FreeFloatingBase which uses Euler angles for its configuration vectors instead of quaternions. In general, users would be advised to stick with DQ_FreeFloatingBase due to the well-known drawbacks of Euler angle representations. However, this class is also made available for the very rare cases where you absolutely cannot get out of using Euler angles (e.g., some other element of your stack uses them for some reason).
- DQ_LeggedRobot - A class for representing legged robots. Every legged robot is comprised of a base (either DQ_FreeFloatingBase or DQ_FreeFloatingBaseEuler) and some legs (represented as DQ_SerialManipulatorDH objects).

Additionally, the Corin Hexapod is also available as an example alongside those robots provided by the original DQ Robotics distribution. New users should see `src/robots/CorinHexapod.cpp` for an example of how the DQ_LeggedRobot constructor may be used to define your own legged robots. The original Corin Hexapod was developed by Wei Cheah for the University of Manchester, more information on Corin can be found [here](https://uomrobotics.com/collaborations/rain/remote-inspection/corin.html).

## To Do

Because this fork is still under development, some functionality is not yet implemented. Functionality which is planned to be implemented, but which has not been yet, includes:
- Functions for the pose Jacobian derivative in DQ_FreeFloatingBase and DQ_LeggedRobot (currently calls to these functions will result in an exception being raised).
- Ensuring that the controllers shipped with DQ_Robotics can be used with DQ_LeggedRobot objects (at time of writing this has not been tested).
