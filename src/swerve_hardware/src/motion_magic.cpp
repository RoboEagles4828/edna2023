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

    // (maxV - curr)t1 + curr
    // (curr - maxV)tf + b = 0
    // 2 10
    // 4(t1)
    

    double MotionMagic::getNextVelocity(const double targetPosition, const double sensorPosition, const double sensorVelocity, const double dt) {
        // method 0
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
        if (error < 0.0) {
            dir = -1.0;
        }
        
        if (absError <= rampWindow1) {
            return velocityInRampWindow1 * dir;
        } else if (absError <= rampWindow2) {
            return velocityInRampWindow2 * dir;
        } else {
            return velocityInCruiseWindow * dir;
        }

        //method 1
        // double displacement = std::abs(getPositionDifference(targetPosition, sensorPosition));
        // double dir = targetPosition - sensorPosition;
        // double slow_down_dist = (MAX_JERK/6) * pow(2*sensorVelocity/MAX_JERK, 1.5);

        // if(std::abs(displacement - 0.0) <= tolerance) return 0.0;

        // if(dir > 0) {
        //     if(displacement <= slow_down_dist) return std::max(sensorVelocity - dt * dt * MAX_JERK, -1*MAX_VELOCITY);
        //     // std::cout<<std::min(sensorVelocity + dt * dt * MAX_JERK, MAX_VELOCITY);
        //     else return std::min(sensorVelocity + dt * dt * MAX_JERK, MAX_VELOCITY);
        // } else {
        //     if(displacement <= slow_down_dist) return std::min(sensorVelocity + dt * dt * MAX_JERK, MAX_VELOCITY);
        //     else return std::max(sensorVelocity - dt * dt * MAX_JERK, -1*MAX_VELOCITY);
        // }
    }
} // namespace swerve_hardware
