
import wpilib
import wpilib.simulation
import ctre
from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface

from sim.talonFxSim import TalonFxSim
from sim.cancoderSim import CancoderSim
from hardware_interface.drivetrain import getAxleRadians, getWheelRadians, SwerveModule, AXLE_JOINT_GEAR_RATIO

import math
import typing

if typing.TYPE_CHECKING:
    from robot import EdnaRobot

WHEEL_MOTOR_MAX_VEL = 20900
WHEEL_MOTOR_ACCEL_TIME = 0.25
AXLE_MOTOR_MAX_VEL = 20900
AXLE_MOTOR_ACCEL_TIME = 0.25

# Calculations
axle_radius = 0.05
axle_mass = 0.23739
wheel_radius = 0.0508
wheel_length = 0.0381
wheel_mass = 0.2313

center_axle_moi = 0.5 * pow(axle_radius, 2) * axle_mass
center_side_wheel_moi = (0.25 * pow(wheel_radius, 2) * wheel_mass) + ((1/12) * pow(wheel_length, 2) * wheel_mass)
center_wheel_moi = 0.5 * pow(wheel_radius, 2) * wheel_mass

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "EdnaRobot"):
        
        self.physics_controller = physics_controller
        
        self.roborio = wpilib.simulation.RoboRioSim()
        self.battery = wpilib.simulation.BatterySim()
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))

        self.frontLeftModuleSim = self.setupSwerveModuleSim(robot.drive_train.front_left)
        self.frontRightModuleSim = self.setupSwerveModuleSim(robot.drive_train.front_right)
        self.rearLeftModuleSim = self.setupSwerveModuleSim(robot.drive_train.rear_left)
        self.rearRightModuleSim = self.setupSwerveModuleSim(robot.drive_train.rear_right)
        

    def setupSwerveModuleSim(self, module: "SwerveModule"):
        ModuleObejectSim = {}
        wheelMOI = center_wheel_moi
        axleMOI = center_axle_moi + center_side_wheel_moi
        # There is a bad feedback loop between controller and the rio code
        # It will create oscillations in the simulation when the robot is not being commanded to move
        # The issue is from the controller commanding the axle position to stay at the same position
        ModuleObejectSim["wheelMotorSim"] = TalonFxSim(module.wheel_motor, wheelMOI, 1, False)
        ModuleObejectSim["axleMotorSim"] = TalonFxSim(module.axle_motor, axleMOI, AXLE_JOINT_GEAR_RATIO, False)
        ModuleObejectSim["encoderSim"] = CancoderSim(module.encoder, module.encoder_offset, True)
        return ModuleObejectSim
    
    def updateSwerveModuleSim(self, moduleSim, tm_diff):
        moduleSim["wheelMotorSim"].update(tm_diff)
        moduleSim["axleMotorSim"].update(tm_diff)
        moduleSim["encoderSim"].update(tm_diff, moduleSim["axleMotorSim"].getVelocityRadians())

    # Useful for debugging the simulation or code
    def printSwerveModuleSim(self, moduleSim):
        wheelPos = round(getWheelRadians(moduleSim["wheelMotorSim"].getSimulatedPosition(), "position"), 2)
        wheelVel = round(getWheelRadians(moduleSim["wheelMotorSim"].getSimulatedVelocity(), "velocity"), 2)
        axlePos = round(getAxleRadians(moduleSim["axleMotorSim"].getSimulatedPosition(), "position"), 2)
        axleVel = round(getAxleRadians(moduleSim["axleMotorSim"].getSimulatedVelocity(), "velocity"), 2)
        encoderPos = round(moduleSim["encoderSim"].getSimulatedPosition(), 2)
        encoderVel = round(moduleSim["encoderSim"].getSimulatedVelocity(), 2)
        print(f"Wheel POS: {wheelPos} VEL: {wheelVel}\t\tEncoder POS: {encoderPos} VEL: {encoderVel}\t\tAxle POS: {axlePos} VEL: {axleVel}")
        # print(moduleSim["wheelMotorSim"].talon.getSimCollection().getMotorOutputLeadVoltage())
    
    def update_sim(self, now: float, tm_diff: float) -> None:

        # Simulate the motor
        self.updateSwerveModuleSim(self.frontLeftModuleSim, tm_diff)
        self.updateSwerveModuleSim(self.frontRightModuleSim, tm_diff)
        self.updateSwerveModuleSim(self.rearLeftModuleSim, tm_diff)
        self.updateSwerveModuleSim(self.rearRightModuleSim, tm_diff)
        self.printSwerveModuleSim(self.frontLeftModuleSim)

        # Add Currents into Battery Simulation
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))
