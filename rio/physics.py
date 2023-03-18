
import wpilib
import wpilib.simulation
from wpilib import RobotController
import ctre
from ctre._ctre import TalonFXSimCollection
from ctre.sensors import CANCoderSimCollection
from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface

from sim.talonFxSim import TalonFxSim
from sim.cancoderSim import CancoderSim
from hardware_interface.drivetrain import getAxleRadians, MODULE_CONFIG

import math
import typing

if typing.TYPE_CHECKING:
    from robot import EdnaRobot

WHEEL_MOTOR_MAX_VEL = 20920
WHEEL_MOTOR_ACCEL_TIME = 0.20
AXLE_MOTOR_MAX_VEL = 21050
AXLE_MOTOR_ACCEL_TIME = 0.20

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "EdnaRobot"):
        
        self.physics_controller = physics_controller
        
        self.roborio = wpilib.simulation.RoboRioSim()
        self.battery = wpilib.simulation.BatterySim()
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))

        self.frontLeftWheelSim = TalonFxSim(robot.drive_train.front_left.wheel_motor, WHEEL_MOTOR_ACCEL_TIME, WHEEL_MOTOR_MAX_VEL, False)
        self.frontLeftAxleSim = TalonFxSim(robot.drive_train.front_left.axle_motor, AXLE_MOTOR_ACCEL_TIME, AXLE_MOTOR_MAX_VEL, False)
        self.frontLeftEncoderSim = CancoderSim(robot.drive_train.front_left.encoder, MODULE_CONFIG["front_left"]["encoder_offset"])

    def update_sim(self, now: float, tm_diff: float) -> None:

        # Simulate the motor
        self.frontLeftWheelSim.update(tm_diff)
        self.frontLeftAxleSim.update(tm_diff)
        # self.frontLeftEncoderSim.update(tm_diff,
        #     getAxleRadians(self.frontLeftAxleSim.velocity, "velocity"),
        #     getAxleRadians(self.frontLeftAxleSim.pos, "position"))


        # Add Currents into Battery Simulation
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))
