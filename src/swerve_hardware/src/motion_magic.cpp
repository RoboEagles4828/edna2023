#include "swerve_hardware/motion_magic.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace swerve_hardware
{

MotionMagic::MotionMagic(double maxAcceleration, double maxVelocity)
{
    this->MAX_ACCELERATION = maxAcceleration;
    this->MAX_VELOCITY = maxVelocity;
}

double MotionMagic::getPositionDifference(double targetPosition, double sensorPosition) {
    double copy_targetPosition = targetPosition;
    double difference = copy_targetPosition - sensorPosition;
    if (difference > M_PI) {
        copy_targetPosition -= 2 * M_PI;
    } else if (difference < -M_PI) {
        copy_targetPosition += 2 * M_PI;
    }
    return std::fmod(copy_targetPosition - sensorPosition, M_PI);
}

double MotionMagic::getNextVelocity(const double targetPosition, const double sensorPosition, const double sensorVelocity, const double dt) {
    // Basic implementation for now
    double error = getPositionDifference(targetPosition, sensorPosition);
    double absError = std::abs(error);
    if (absError < tolerance) {
        return 0.0;
    }
    
    double dir = 1.0;
    if (error < 0) dir = -1.0;

    if (absError <= rampWindow1) {
        return velocityInRampWindow1 * dir;
    } else if (absError <= rampWindow2) {
        return velocityInRampWindow2 * dir;
    } else {
        return velocityInCruiseWindow * dir;
    }
}

} // namespace swerve_hardware
