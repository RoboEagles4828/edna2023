#ifndef SWERVE_HARDWARE__MOTION_MAGIC_HPP_
#define SWERVE_HARDWARE__MOTION_MAGIC_HPP_

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include "swerve_hardware/visibility_control.h"

namespace swerve_hardware
{

class MotionMagic
{
public:
    SWERVE_HARDWARE_PUBLIC
    MotionMagic(double maxAcceleration, double maxVelocity);

    SWERVE_HARDWARE_PUBLIC
    double getNextVelocity(const double targetPosition, const double sensorPosition, const double sensorVelocity, const double dt);

    SWERVE_HARDWARE_PUBLIC
    double getPositionDifference(double targetPosition, double sensorPosition);

    SWERVE_HARDWARE_PUBLIC
    double getTotalTime(double targetPosition);

private:
    double MAX_ACCELERATION;
    double MAX_VELOCITY;
    double MAX_JERK = 6 * M_PI;
    double prevVel = 0.0;
    double prevAcceleration = 0.0;
    double prevError = 0.0;
    double prevTargetPosition = 0.0;
    double totalDistance = 0.0;
    double zeroTime = 0.0;

    double tolerance = 0.15;
    double rampWindow1 = 0.3;
    double rampWindow2 = 0.8;
    double velocityInRampWindow1 = 0.1;
    double velocityInRampWindow2 = 2.0;
    double velocityInCruiseWindow = 3.0;
};

}  // namespace swerve_hardware

#endif  // SWERVE_HARDWARE__MOTION_MAGIC_HPP_
