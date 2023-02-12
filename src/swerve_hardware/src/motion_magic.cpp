#include "swerve_hardware/motion_magic.hpp"
#include <cmath>

namespace swerve_hardware
{

MotionMagic::MotionMagic(double maxAcceleration, double maxVelocity)
{
    this->MAX_ACCELERATION = maxAcceleration;
    this->MAX_VELOCITY = maxVelocity;
}

double MotionMagic::getPositionDifference(double targetPosition, double sensorPosition) {
    double difference = targetPosition - sensorPosition;
    if (difference > M_PI) {
        targetPosition -= 2 * M_PI;
    } else if (difference < -M_PI) {
        targetPosition += 2 * M_PI;
    }
    return targetPosition - sensorPosition;
}

double MotionMagic::getNextVelocity(double targetPosition, double sensorPosition, double sensorVelocity, double dt) {
    // Calculate an error term
    double error = getPositionDifference(targetPosition, sensorPosition);
    if (abs(error) < tolerance) {
        return 0.0;
    }
    
    double velocity = 0.0;
    return velocity;
}

}