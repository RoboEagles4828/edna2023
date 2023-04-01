
import wpilib
import wpilib.simulation
import ctre
from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface

from sim.talonFxSim import TalonFxSim
from sim.cancoderSim import CancoderSim
from hardware_interface.drivetrain import getAxleRadians, getWheelRadians, SwerveModule, AXLE_JOINT_GEAR_RATIO
from hardware_interface.armcontroller import PORTS, TOTAL_INTAKE_REVOLUTIONS
from hardware_interface.joystick import CONTROLLER_PORT

import math
import typing

if typing.TYPE_CHECKING:
    from robot import EdnaRobot

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
        
        self.xbox = wpilib.simulation.XboxControllerSim(CONTROLLER_PORT)
        self.roborio = wpilib.simulation.RoboRioSim()
        self.battery = wpilib.simulation.BatterySim()
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))

        self.frontLeftModuleSim = SwerveModuleSim(robot.drive_train.front_left)
        self.frontRightModuleSim = SwerveModuleSim(robot.drive_train.front_right)
        self.rearLeftModuleSim = SwerveModuleSim(robot.drive_train.rear_left)
        self.rearRightModuleSim = SwerveModuleSim(robot.drive_train.rear_right)

        self.elevator = TalonFxSim(robot.arm_controller.elevator.motor, 0.0003, 1, False)
        # self.intake = TalonFxSim(robot.arm_controller.bottom_gripper_lift.motor, 0.0004, 1, False)
        # self.intake.addLimitSwitch("fwd", 0)
        # self.intake.addLimitSwitch("rev", TOTAL_INTAKE_REVOLUTIONS * -2 * math.pi)

        self.pneumaticHub = wpilib.simulation.REVPHSim(PORTS['HUB'])
        self.armRollerBar = wpilib.simulation.DoubleSolenoidSim(self.pneumaticHub, *PORTS['ARM_ROLLER_BAR'])
        self.topGripperSlider = wpilib.simulation.DoubleSolenoidSim(self.pneumaticHub, *PORTS['TOP_GRIPPER_SLIDER'])
        self.topGripper = wpilib.simulation.DoubleSolenoidSim(self.pneumaticHub, *PORTS['TOP_GRIPPER'])


    def update_sim(self, now: float, tm_diff: float) -> None:

        # Simulate Swerve Modules
        self.frontLeftModuleSim.update(tm_diff)
        self.frontRightModuleSim.update(tm_diff)
        self.rearLeftModuleSim.update(tm_diff)
        self.rearRightModuleSim.update(tm_diff)

        # Simulate Arm
        self.elevator.update(tm_diff)
        # self.intake.update(tm_diff)

        # Add Currents into Battery Simulation
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))


class SwerveModuleSim():
    wheel : TalonFxSim = None
    axle : TalonFxSim = None
    encoder : CancoderSim = None

    def __init__(self, module: "SwerveModule"):
        wheelMOI = center_wheel_moi
        axleMOI = center_axle_moi + center_side_wheel_moi
        self.wheel = TalonFxSim(module.wheel_motor, wheelMOI, 1, False)
        self.axle = TalonFxSim(module.axle_motor, axleMOI, AXLE_JOINT_GEAR_RATIO, False)
        self.encoder = CancoderSim(module.encoder, module.encoder_offset, True)
        # There is a bad feedback loop between controller and the rio code
        # It will create oscillations in the simulation when the robot is not being commanded to move
        # The issue is from the controller commanding the axle position to stay at the same position when idle
        # but if the axle is moving during that time it will constantly overshoot the idle position
    
    def update(self, tm_diff):
        self.wheel.update(tm_diff)
        self.axle.update(tm_diff)
        self.encoder.update(tm_diff, self.axle.getVelocityRadians())
    
    # Useful for debugging the simulation or code
    def __str__(self) -> str:
        wheelPos = getWheelRadians(self.wheel.talon.getSelectedSensorPosition(), "position")
        wheelVel = getWheelRadians(self.wheel.talon.getSelectedSensorVelocity(), "velocity")
        stateStr = f"Wheel POS: {wheelPos:5.2f} VEL: {wheelVel:5.2f} "
        
        axlePos = getAxleRadians(self.axle.talon.getSelectedSensorPosition(), "position")
        axleVel = getAxleRadians(self.axle.talon.getSelectedSensorVelocity(), "velocity")
        stateStr += f"Axle POS: {axlePos:5.2f} VEL: {axleVel:5.2f} "
        
        encoderPos = math.radians(self.encoder.cancoder.getAbsolutePosition())
        encoderVel = math.radians(self.encoder.cancoder.getVelocity())
        stateStr += f"Encoder POS: {encoderPos:5.2f} VEL: {encoderVel:5.2f}"
        return stateStr