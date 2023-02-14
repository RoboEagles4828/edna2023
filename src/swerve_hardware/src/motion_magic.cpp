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
        // double error = targetPosition - sensorPosition;
        double absError = std::abs(error);
        if (targetPosition != prevTargetPosition) {
                // prevVelocity = 0.0;
                // prevAcceleration = 0.0;
                // prevError = 0.0;
                totalDistance = absError;
                zeroTime = clock();
                prevTargetPosition = targetPosition;
            }
        if (absError < tolerance) {
            return 0.0;
        }
        else {
            double dir = 1.0;
            if (error < 0) dir = -1.0;
            double threshold = (1.0/3.0) * totalDistance;
            // double elapsedTime = (clock() - zeroTime) / CLOCKS_PER_SEC;
            // double totalTime = getTotalTime(totalDistance);
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
                return (sensorVelocity - MAX_ACCELERATION) * dir * dt;
            }
            else {        
                return 0.0;
            }

            // std::cout << "TotalTime: " << totalTime << std::endl;
            // std::cout << "ElapsedTime: " << elapsedTime << std::endl;
            // if (elapsedTime <= (1.0/3.0) * totalTime) {
            //     return sensorVelocity + MAX_ACCELERATION * dir * dt;
            // } 
            // else if (elapsedTime <= (2.0/3.0) * totalTime) {
            //     return MAX_VELOCITY * dir * dt;
            // } 
            // else if (elapsedTime < totalTime) {
            //     return sensorVelocity - MAX_ACCELERATION * dir * dt;
            // }
            // else if (elapsedTime == totalTime) {
            //     return 0.0;
            // }
            // else {
            //     return 0.0;
            // }
            
        }

        // if (absError <= rampWindow1) {
        //     return velocityInRampWindow1 * dir;
        // } else if (absError <= rampWindow2) {
        //     return velocityInRampWindow2 * dir;
        // } else {
        //     return velocityInCruiseWindow * dir;
        // }
    }

    double MotionMagic::getTotalTime(double targetPosition) {
        //calculation time for accelerating portions of motion profile
        double accelTime = MAX_VELOCITY / MAX_ACCELERATION;

        //calculation time for cruise portion of motion profile
        double cruiseTime = (targetPosition - (MAX_VELOCITY * MAX_VELOCITY / MAX_ACCELERATION)) / MAX_VELOCITY;

        return accelTime + cruiseTime;
    }
} // namespace swerve_hardware
