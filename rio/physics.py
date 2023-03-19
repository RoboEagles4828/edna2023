
import wpilib
import wpilib.simulation
import ctre
from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface

from sim.talonFxSim import TalonFxSim
from sim.cancoderSim import CancoderSim
from hardware_interface.drivetrain import getAxleRadians, SwerveModule

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

        self.frontLeftModuleSim = self.setupSwerveModuleSim(robot.drive_train.front_left)
        self.frontRightModuleSim = self.setupSwerveModuleSim(robot.drive_train.front_right)
        self.rearLeftModuleSim = self.setupSwerveModuleSim(robot.drive_train.rear_left)
        self.rearRightModuleSim = self.setupSwerveModuleSim(robot.drive_train.rear_right)
        

    def setupSwerveModuleSim(self, module: "SwerveModule"):
        ModuleObejectSim = {}
        ModuleObejectSim["wheelMotorSim"] = TalonFxSim(module.wheel_motor, WHEEL_MOTOR_ACCEL_TIME, WHEEL_MOTOR_MAX_VEL, False)
        ModuleObejectSim["axleMotorSim"] = TalonFxSim(module.axle_motor, AXLE_MOTOR_ACCEL_TIME, AXLE_MOTOR_MAX_VEL, False)
        ModuleObejectSim["encoderSim"] = CancoderSim(module.encoder, module.encoder_offset, True)
        return ModuleObejectSim
    
    def updateSwerveModuleSim(self, moduleSim, tm_diff):
        moduleSim["wheelMotorSim"].update(tm_diff)
        moduleSim["axleMotorSim"].update(tm_diff)
        moduleSim["encoderSim"].update(tm_diff, 
            getAxleRadians(moduleSim["axleMotorSim"].deltaPos, "position"), 
            getAxleRadians(moduleSim["axleMotorSim"].velocity, "velocity"))

    def printSwerveModuleSim(self, moduleSim):
        axlePos = round(getAxleRadians(moduleSim["axleMotorSim"].getSimulatedPosition(), "position"), 3)
        axleVel = round(getAxleRadians(moduleSim["axleMotorSim"].getSimulatedVelocity(), "velocity"), 3)
        encoderPos = round(moduleSim["encoderSim"].getSimulatedPosition(), 3)
        encoderVel = round(moduleSim["encoderSim"].getSimulatedVelocity(), 3)
        print(f"Encoder POS: {encoderPos} VEL: {encoderVel}\t\tAxle POS: {axlePos} VEL: {axleVel}")
    
    def update_sim(self, now: float, tm_diff: float) -> None:

        # Simulate the motor
        self.updateSwerveModuleSim(self.frontLeftModuleSim, tm_diff)
        self.updateSwerveModuleSim(self.frontRightModuleSim, tm_diff)
        self.updateSwerveModuleSim(self.rearLeftModuleSim, tm_diff)
        self.updateSwerveModuleSim(self.rearRightModuleSim, tm_diff)
        self.printSwerveModuleSim(self.frontLeftModuleSim)

        # Add Currents into Battery Simulation
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))
