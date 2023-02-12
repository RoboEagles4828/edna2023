#ifndef SWERVE_HARDWARE__MOTION_MAGIC_HPP_
#define SWERVE_HARDWARE__MOTION_MAGIC_HPP_

#include "swerve_hardware/visibility_control.h"

namespace swerve_hardware
{

class MotionMagic
{
public:
    SWERVE_HARDWARE_PUBLIC
    MotionMagic(double maxAcceleration, double maxVelocity);

    SWERVE_HARDWARE_PUBLIC
    double getNextVelocity(double targetPosition, double sensorPosition, double sensorVelocity, double dt);

    SWERVE_HARDWARE_PUBLIC
    double getPositionDifference(double targetPosition, double sensorPosition);

private:
    double MAX_ACCELERATION;
    double MAX_VELOCITY;
    double prevVelocity = 0.0;
    double prevAcceleration = 0.0;
    double prevError = 0.0;
    double tolerance = 0.05;
};

}

#endif  // SWERVE_HARDWARE__DIFFBOT_SYSTEM_HPP_