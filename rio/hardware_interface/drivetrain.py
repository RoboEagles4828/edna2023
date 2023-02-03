import wpilib
import ctre
from enum import Enum, auto
import math
import time
import logging

# Small Gear Should Face the back of the robot
# All wheel drive motors should not be inverted
# All axle turn motors should be inverted + sensor phase
# All Cancoders should be direction false
MODULE_CONFIG = {
    "front_left": {
        "wheel_joint_name": "front_left_wheel_joint",
        "wheel_motor_port": 9,
        "axle_joint_name": "front_left_axle_joint",
        "axle_motor_port": 7,
        "axle_encoder_port": 8,
        "encoder_offset": 19.951,
    },
    "front_right": {
        "wheel_joint_name": "front_right_wheel_joint",
        "wheel_motor_port": 12,
        "axle_joint_name": "front_right_axle_joint",
        "axle_motor_port": 10,
        "axle_encoder_port": 11,
        "encoder_offset": 254.971,
    },
    "rear_left": {
        "wheel_joint_name": "rear_left_wheel_joint",
        "wheel_motor_port": 6,
        "axle_joint_name": "rear_left_axle_joint",
        "axle_motor_port": 4,
        "axle_encoder_port": 5,
        "encoder_offset": 304.189,
    },
    "rear_right": {
        "wheel_joint_name": "rear_right_wheel_joint",
        "wheel_motor_port": 3,
        "axle_joint_name": "rear_right_axle_joint",
        "axle_motor_port": 1,
        "axle_encoder_port": 2,
        "encoder_offset": 68.379,
    }
}
AXLE_DIRECTION = True
WHEEL_DIRECTION = False
WHEEL_JOINT_GEAR_RATIO = 6.75 #8.14
AXLE_JOINT_GEAR_RATIO = 150.0/7.0
TICKS_PER_REV = 2048.0
CMD_TIMEOUT_SECONDS = 1

nominal_voltage = 12.0
steer_current_limit = 20.0

# This is needed to fix bad float values being published by RTI from the RIO.
# To fix this, we scale the float and convert to integers. 
# Then we scale it back down inside the ROS2 hardware interface.
SCALING_FACTOR_FIX = 10000

# Encoder Constants
ENCODER_DIRECTION = False
encoder_ticks_per_rev = 4096.0
encoder_reset_velocity = math.radians(0.5)
encoder_reset_iterations = 500

axle_pid_constants = {
    "kP": 0.2,
    "kI": 0.0,
    "kD": 0.1,
    "kF": 0.2,
    "kIzone": 0,
    "kPeakOutput": 1.0
}
wheel_pid_constants = {
        "kF": 1023.0/20660.0,
        "kP": 0.1,
        "kI": 0.001,
        "kD": 5
}
slot_idx = 0
pid_loop_idx = 0
timeout_ms = 30

velocityConstant = 1.0
accelerationConstant = 0.5

# Conversion Functions
positionCoefficient = 2.0 * math.pi / TICKS_PER_REV / AXLE_JOINT_GEAR_RATIO
velocityCoefficient = positionCoefficient * 10.0
# axle (radians) -> shaft (ticks)
def getShaftTicks(radians, type):
    if type == "position":
        return radians / positionCoefficient
    elif type == "velocity":
        return radians / velocityCoefficient
    else:
        return 0

# shaft (ticks) -> axle (radians)
def getAxleRadians(ticks, type):
    if type == "position":
        return ticks * positionCoefficient
    elif type == "velocity":
        return ticks * velocityCoefficient
    else:
        return 0

wheelPositionCoefficient = 2.0 * math.pi / TICKS_PER_REV / WHEEL_JOINT_GEAR_RATIO
wheelVelocityCoefficient = wheelPositionCoefficient * 10.0
# wheel (radians) -> shaft (ticks)
def getWheelShaftTicks(radians, type):
    if type == "position":
        return radians / wheelPositionCoefficient
    elif type == "velocity":
        return radians / wheelVelocityCoefficient
    else:
        return 0

# shaft (ticks) -> wheel (radians)
def getWheelRadians(ticks, type):
    if type == "position":
        return ticks * wheelPositionCoefficient
    elif type == "velocity":
        return ticks * wheelVelocityCoefficient
    else:
        return 0


class SwerveModule():
    
    def __init__(self, module_config) -> None:
        #IMPORTANT:
        # The wheel joint is the motor that drives the wheel.
        # The axle joint is the motor that steers the wheel.
        
        self.wheel_joint_name = module_config["wheel_joint_name"]
        self.axle_joint_name = module_config["axle_joint_name"]
        self.wheel_joint_port = module_config["wheel_motor_port"]
        self.axle_joint_port = module_config["axle_motor_port"]
        self.axle_encoder_port = module_config["axle_encoder_port"]
        self.encoder_offset = module_config["encoder_offset"]

        self.wheel_motor = ctre.TalonFX(self.wheel_joint_port)
        self.axle_motor = ctre.TalonFX(self.axle_joint_port)
        self.encoder = ctre.CANCoder(self.axle_encoder_port)

        self.last_wheel_vel_cmd = None
        self.last_axle_vel_cmd = None
        self.reset_iterations = 0
        
        self.setupEncoder()
        self.setupWheelMotor()
        self.setupAxleMotor()
        

    def setupEncoder(self):
        self.encoderconfig = ctre.CANCoderConfiguration()
        self.encoderconfig.absoluteSensorRange = ctre.AbsoluteSensorRange.Unsigned_0_to_360
        self.encoderconfig.initializationStrategy = ctre.SensorInitializationStrategy.BootToAbsolutePosition
        self.encoderconfig.sensorDirection = ENCODER_DIRECTION
        self.encoder = ctre.CANCoder(self.axle_encoder_port)
        self.encoder.configAllSettings(self.encoderconfig)
        self.encoder.setPositionToAbsolute(timeout_ms)
        self.encoder.setStatusFramePeriod(ctre.CANCoderStatusFrame.SensorData, 10, timeout_ms)
    
    def getEncoderPosition(self):
        return math.radians(self.encoder.getAbsolutePosition() - self.encoder_offset)
    
    def getEncoderVelocity(self):
        return math.radians(self.encoder.getVelocity())
    
    def setupWheelMotor(self):
        self.wheel_motor.configFactoryDefault()
        self.wheel_motor.configNeutralDeadband(0.01)
        self.wheel_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, timeout_ms)
        self.wheel_motor.configNominalOutputForward(0, timeout_ms)
        self.wheel_motor.configNominalOutputReverse(0, timeout_ms)
        self.wheel_motor.configPeakOutputForward(1, timeout_ms)
        self.wheel_motor.configPeakOutputReverse(-1, timeout_ms)
        self.wheel_motor.setSensorPhase(WHEEL_DIRECTION)
        self.wheel_motor.setInverted(WHEEL_DIRECTION)
        self.wheel_motor.config_kF(0, wheel_pid_constants["kF"], timeout_ms)
        self.wheel_motor.config_kP(0, wheel_pid_constants["kP"], timeout_ms)
        self.wheel_motor.config_kI(0, wheel_pid_constants["kI"], timeout_ms)
        self.wheel_motor.config_kD(0, wheel_pid_constants["kD"], timeout_ms)
        self.wheel_motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_1_General, 10, timeout_ms)
        self.wheel_motor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def setupAxleMotor(self):
        self.axle_motor.configFactoryDefault()
        self.axle_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, pid_loop_idx, timeout_ms)
        self.axle_motor.configNeutralDeadband(0.01, timeout_ms)
        self.axle_motor.setSensorPhase(AXLE_DIRECTION)
        self.axle_motor.setInverted(AXLE_DIRECTION)
        self.axle_motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout_ms)
        self.axle_motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout_ms)
        self.axle_motor.configNominalOutputForward(0, timeout_ms)
        self.axle_motor.configNominalOutputReverse(0, timeout_ms)
        self.axle_motor.configPeakOutputForward(1, timeout_ms)
        self.axle_motor.configPeakOutputReverse(-1, timeout_ms)
        self.axle_motor.selectProfileSlot(slot_idx, pid_loop_idx)
        self.axle_motor.config_kP(slot_idx, axle_pid_constants["kP"], timeout_ms)
        self.axle_motor.config_kI(slot_idx, axle_pid_constants["kI"], timeout_ms)
        self.axle_motor.config_kD(slot_idx, axle_pid_constants["kD"], timeout_ms)
        # TODO: Figure out Magic Numbers and Calculations Below
        self.axle_motor.config_kF(slot_idx, (1023.0 *  velocityCoefficient / nominal_voltage) * velocityConstant, timeout_ms)
        self.axle_motor.configMotionCruiseVelocity(2.0 / velocityConstant / velocityCoefficient, timeout_ms)
        self.axle_motor.configMotionAcceleration((8.0 - 2.0) / accelerationConstant / velocityCoefficient, timeout_ms)
        self.axle_motor.configMotionSCurveStrength(1)
        self.axle_motor.setSelectedSensorPosition(getShaftTicks(self.getEncoderPosition(), "position"), pid_loop_idx, timeout_ms)
        self.axle_motor.configVoltageCompSaturation(nominal_voltage, timeout_ms)
        currentLimit = ctre.SupplyCurrentLimitConfiguration()
        currentLimit.enable = True
        currentLimit.currentLimit = steer_current_limit
        self.axle_motor.configSupplyCurrentLimit(currentLimit, timeout_ms)
        self.axle_motor.enableVoltageCompensation(True)
        self.axle_motor.setNeutralMode(ctre.NeutralMode.Brake)


    def set(self, wheel_motor_vel, axle_position):
        wheel_vel = getWheelShaftTicks(wheel_motor_vel, "velocity")
        self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, wheel_vel)
        self.last_wheel_vel_cmd = wheel_vel

        # MOTION MAGIC CONTROL FOR AXLE POSITION
        axle_motorPosition = getAxleRadians(self.axle_motor.getSelectedSensorPosition(), "position")
        axle_motorVelocity = getAxleRadians(self.axle_motor.getSelectedSensorVelocity(), "velocity")
        axle_absolutePosition = self.getEncoderPosition()

        # Reset
        if axle_motorVelocity < encoder_reset_velocity:
            self.reset_iterations += 1
            if self.reset_iterations >= encoder_reset_iterations:
                self.reset_iterations = 0
                self.axle_motor.setSelectedSensorPosition(getShaftTicks(axle_absolutePosition, "position"))
                axle_motorPosition = axle_absolutePosition
        else:
            self.reset_iterations = 0

        # First let's assume that we will move directly to the target position.
        newAxlePosition = axle_position

        # The motor could get to the target position by moving clockwise or counterclockwise.
        # The shortest path should be the direction that is less than pi radians away from the current motor position.
        # The shortest path could loop around the circle and be less than 0 or greater than 2pi.
        # We need to get the absolute current position to determine if we need to loop around the 0 - 2pi range.
        
        # The current motor position does not stay inside the 0 - 2pi range.
        # We need the absolute position to compare with the target position.
        axle_absoluteMotorPosition = math.fmod(axle_motorPosition, 2.0 * math.pi)
        if axle_absoluteMotorPosition < 0.0:
            axle_absoluteMotorPosition += 2.0 * math.pi

        # If the target position was in the first quadrant area 
        # and absolute motor position was in the last quadrant area
        # then we need to move into the next loop around the circle.
        if newAxlePosition - axle_absoluteMotorPosition < -math.pi:
            newAxlePosition += 2.0 * math.pi
        # If the target position was in the last quadrant area
        # and absolute motor position was in the first quadrant area
        # then we need to move into the previous loop around the circle.
        elif newAxlePosition - axle_absoluteMotorPosition > math.pi:
            newAxlePosition -= 2.0 * math.pi

        # Last, add the current existing loops that the motor has gone through.
        newAxlePosition += axle_motorPosition - axle_absoluteMotorPosition
        self.axle_motor.set(ctre.TalonFXControlMode.MotionMagic, getShaftTicks(newAxlePosition, "position"))


    def stop(self):
        self.wheel_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.axle_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def getEncoderData(self):
        output = [
            {
                "name": self.wheel_joint_name,
                "position": int(getWheelRadians(self.wheel_motor.getSelectedSensorPosition(), "position") * SCALING_FACTOR_FIX),
                "velocity": int(getWheelRadians(self.wheel_motor.getSelectedSensorVelocity(), "velocity") * SCALING_FACTOR_FIX)
            },
            {
                "name": self.axle_joint_name,
                "position": int(self.getEncoderPosition() * SCALING_FACTOR_FIX),
                "velocity": int(self.getEncoderVelocity() * SCALING_FACTOR_FIX)
            }
        ]
        return output

class DriveTrain():
    def __init__(self):
        self.last_cmds_time = time.time()
        self.warn_timeout = True
        self.front_left = SwerveModule(MODULE_CONFIG["front_left"])
        self.front_right = SwerveModule(MODULE_CONFIG["front_right"])
        self.back_left = SwerveModule(MODULE_CONFIG["rear_left"])
        self.back_right = SwerveModule(MODULE_CONFIG["rear_right"])
        self.module_lookup = \
        {
            'front_left_axle_joint': self.front_left,
            'front_right_axle_joint': self.front_right,
            'rear_left_axle_joint': self.back_left,
            'rear_right_axle_joint': self.back_right,

        }

    def getEncoderData(self):
        names = [""]*8
        positions = [0]*8
        velocities = [0]*8
        encoderInfo = []
        encoderInfo += self.front_left.getEncoderData() 
        encoderInfo += self.front_right.getEncoderData()
        encoderInfo += self.back_left.getEncoderData()
        encoderInfo += self.back_right.getEncoderData()
        assert len(encoderInfo) == 8
        for index, encoder in enumerate(encoderInfo):
            names[index] = encoder['name']
            positions[index] = encoder['position']
            velocities[index] = encoder['velocity']
        return { "name": names, "position": positions, "velocity": velocities }

    def stop(self):
        self.front_left.stop()
        self.front_right.stop()
        self.back_left.stop()
        self.back_right.stop()


    def sendCommands(self, commands):
        if commands:
            self.last_cmds = commands
            self.last_cmds_time = time.time()
            self.warn_timeout = True
            for i in range(len(commands['name'])):
                if 'axle' in commands['name'][i]:
                    axle_name = commands['name'][i]
                    axle_position = commands['position'][i] if len(commands['position']) > i else 0.0

                    wheel_name = axle_name.replace('axle', 'wheel')
                    wheel_index = commands['name'].index(wheel_name)
                    wheel_velocity = commands['velocity'][wheel_index] if len(commands['velocity']) > wheel_index else 0.0

                    module = self.module_lookup[axle_name]
                    module.set(wheel_velocity, axle_position)
                    # logging.info(f"{wheel_name}: {wheel_velocity}\n{axle_name}: {axle_position}")
        else:
            current_time = time.time()
            if current_time - self.last_cmds_time > CMD_TIMEOUT_SECONDS:
                self.stop()
                # Display Warning Once
                if self.warn_timeout:
                    logging.warning("CMD TIMEOUT: HALTING")
                    self.warn_timeout = False

