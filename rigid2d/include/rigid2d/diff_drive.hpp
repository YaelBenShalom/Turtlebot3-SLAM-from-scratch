#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for the kinematics of a differential drive robot with a given wheel base and wheel radius.

#include "rigid2d/rigid2d.hpp"

#include <iosfwd>
#include <cmath>


namespace rigid2d
{
    /// \brief The class models the kinematics of a differential drive robot with a given wheel base and wheel radius. The class:
    /// 1. Track the configuration  of a differential-drive robot
    /// 2. Convert a desired twist to the equivalent wheel velocities required to achieve that twist
    /// 3. Update the configuration of the robot, given updated wheel angles (assuming constant wheel velocity in-between updates)
    class DiffDrive
    {
    private:


    public:

    };


}

#endif