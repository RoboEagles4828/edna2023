
import wpilib
import wpilib.simulation
from wpilib import RobotController

import ctre
from ctre._ctre import TalonFXSimCollection
from ctre.sensors import CANCoderSimCollection

from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface

from sim.talonFxSim import TalonFxSim

import math
import typing

if typing.TYPE_CHECKING:
    from robot import edna_robot

WHEEL_MOTOR_MAX_VEL = 0.75
WHEEL_MOTOR_ACCEL_TIME = 5100
AXLE_MOTOR_MAX_VEL = 0.75
AXLE_MOTOR_ACCEL_TIME = 5100

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "edna_robot"):
        
        self.physics_controller = physics_controller
        
        self.roborio = wpilib.simulation.RoboRioSim()
        self.battery = wpilib.simulation.BatterySim()
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))

        # self.frontLeftWheelSim = TalonFxSim(robot.drive_train.front_left.wheel_motor, WHEEL_MOTOR_ACCEL_TIME, WHEEL_MOTOR_MAX_VEL, False)
        # self.frontLeftAxleSim = TalonFxSim(robot.drive_train.front_left.axle_motor.getSimCollection(), AXLE_MOTOR_ACCEL_TIME, AXLE_MOTOR_MAX_VEL, True)
        # self.frontLeftEncoderSim : CANCoderSimCollection = robot.drive_train.front_left.encoder.getSimCollection()

    def update_sim(self, now: float, tm_diff: float) -> None:

        # self.frontLeftWheelSim.setBusVoltage(RobotController.getBatteryVoltage())
        # self.frontLeftAxleSim.setBusVoltage(RobotController.getBatteryVoltage())

        # Simulate the motor
        # self.frontLeftWheelSim.update(tm_diff)

        # Add Currents into Battery Simulation
        currents = [0.0]
        self.roborio.setVInVoltage(self.battery.calculate(currents))
