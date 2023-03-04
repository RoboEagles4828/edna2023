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
        // double error = getPositionDifference(targetPosition, sensorPosition);
        // double absError = std::abs(error);
        // if (targetPosition != prevTargetPosition) {
        //         totalDistance = absError;
        //         prevTargetPosition = targetPosition;
        // }
        // if (absError < tolerance) {
        //     return 0.0;
        // }

        // double dir = 1.0;
        // if (error < 0.0) {
        //     dir = -1.0;
        // }
        
        // if (absError <= rampWindow1) {
        //     return velocityInRampWindow1 * dir;
        // } else if (absError <= rampWindow2) {
        //     return velocityInRampWindow2 * dir;
        // } else {
        //     return velocityInCruiseWindow * dir;
        // }

        //method 1
        
        //Put positions in 2pi mode so no negatives. We add 2pi if the position is negative otherwise its fine.
        double targetPos = targetPosition < 0 ? targetPosition + 2*M_PI : targetPosition;
        double sensorPos = sensorPosition < 0 ? sensorPosition + 2*M_PI : sensorPosition;

        //dist1 is the distance when going through theta=0, and dist2 is not going through theta=0
        double dist1 = std::abs(2*M_PI - std::max(targetPos, sensorPos) + std::min(targetPosition, sensorPosition));
        double dist2 = std::abs(targetPos - sensorPos);

        //we want to choose the minimum distance to move
        double displacement = std::min(dist1, dist2);

        //find the direction you will be moving in, 1 = positive direction, -1 = negative direction
        double dir;
        if(std::abs(displacement - dist1) <= tolerance) { //if the shortest distance is the one going through theta=0
            if(targetPos > sensorPos) dir = -1; 
            else dir = 1;
        } else if(std::abs(displacement - dist2) <= tolerance) { //if the shortest distance is not going through theta=0
            if(targetPos > sensorPos) dir = 1;
            else dir = -1;
        }

        //this is the amount of distance you will have to travel to go to 0 velocity from current velocity 
        //(it is the right part of the velocity trapezoid graph)
        double slow_down_dist = (MAX_JERK/6) * pow(2*sensorVelocity/MAX_JERK, 1.5);

        //if there is no difference between current position and target position, then we dont move
        if(std::abs(displacement) <= tolerance) return 0.0;

        if(dir == 1) { //if you are going in the positive direction
            //velocity shouldnt be negative if going positive direction, so increment
            if(sensorVelocity <= 0) return std::min(sensorVelocity + dt * dt * MAX_JERK, MAX_VELOCITY);
            else {
                //if the target position is very close (time to slow down) and going positive direction, so decrement
                if(displacement <= slow_down_dist) return std::max(sensorVelocity - dt * dt * MAX_JERK, -1*MAX_VELOCITY);
                //if target position is far, then increment
                else return std::min(sensorVelocity + dt * dt * MAX_JERK, MAX_VELOCITY);
            }
        } else { //if you are going in the negative direction
            //velocity shouldnt be positive if going negative direction, so decrement
            if(sensorVelocity >= 0) return std::max(sensorVelocity - dt * dt * MAX_JERK, -1*MAX_VELOCITY);
            else {
                //if the target position is very close (time to slow down) and going negative direction, so increment
                if(displacement <= slow_down_dist) return std::min(sensorVelocity + dt * dt * MAX_JERK, MAX_VELOCITY);
                //if target position is far, then decrement
                else return std::max(sensorVelocity - dt * dt * MAX_JERK, -1*MAX_VELOCITY);
            }
        }
    }
}
