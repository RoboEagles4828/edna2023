#include "swerve_hardware/motion_magic.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <ostream>
#include <iostream>

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
        if (targetPosition != prevTargetPosition) {
                totalDistance = absError;
                prevTargetPosition = targetPosition;
            }
        if (absError < tolerance) {
            return 0.0;
        }
        double dir = 1.0;
        if (error < 0) dir = -1.0;
        double threshold = (1.0/3.0) * totalDistance;
        std::cout << "Error: " << absError << std::endl;
        if (absError >= 2*threshold) {
            std::cout << "Acceleration Period" << std::endl;
            return (sensorVelocity + MAX_ACCELERATION) * dir * dt;
        } 
        else if (absError > threshold && absError < 2*threshold) {
            std::cout << "Cruise Period" << " " << threshold << std::endl;
            return MAX_VELOCITY * dir * dt;
        } 
        else if (absError >= 0 && absError <= tolerance) {
            std::cout << "STOP" << std::endl;
            return 0.0;
        }
        else if (absError < threshold) {
            std::cout << "Deceleration Period" << " " << threshold << std::endl;
            double vel = (sensorVelocity - MAX_ACCELERATION) * dt;
            if (vel <= 0) vel = 0;
            return vel;
        }
        else {        
            return 0.0;
        }
        // if (absError <= rampWindow1) {
        //     return velocityInRampWindow1 * dir;
        // } else if (absError <= rampWindow2) {
        //     return velocityInRampWindow2 * dir;
        // } else {
        //     return velocityInCruiseWindow * dir;
        // }
    }
} // namespace swerve_hardware
