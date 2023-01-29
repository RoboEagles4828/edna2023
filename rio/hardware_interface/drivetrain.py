import wpilib
import ctre
from enum import Enum, auto
import math
import time
import logging
   

MODULE_CONFIG = {
    "front_left": {
        "wheel_joint_name": "front_left_wheel_joint",
        "wheel_motor_port": 9,
        "axle_joint_name": "front_left_axle_joint",
        "axle_motor_port": 7,
        "axle_encoder_port": 8
    },
    "front_right": {
        "wheel_joint_name": "front_right_wheel_joint",
        "wheel_motor_port": 12,
        "axle_joint_name": "front_right_axle_joint",
        "axle_motor_port": 10,
        "axle_encoder_port": 11
    },
    "rear_left": {
        "wheel_joint_name": "rear_left_wheel_joint",
        "wheel_motor_port": 6,
        "axle_joint_name": "rear_left_axle_joint",
        "axle_motor_port": 4,
        "axle_encoder_port": 5
    },
    "rear_right": {
        "wheel_joint_name": "rear_right_wheel_joint",
        "wheel_motor_port": 3,
        "axle_joint_name": "rear_right_axle_joint",
        "axle_motor_port": 1,
        "axle_encoder_port": 2
    }
}
WHEEL_JOINT_GEAR_RATIO = 6.75 #8.14
AXLE_JOINT_GEAR_RATIO = 150.0/7.0
TICKS_PER_REV = 2048
TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)
CMD_TIMEOUT_SECONDS = 1

# This is needed to fix bad float values being published by RTI from the RIO.
# To fix this, we scale the float and convert to integers. 
# Then we scale it back down inside the ROS2 hardware interface.
SCALING_FACTOR_FIX = 10000 


class MotorType(Enum):
    wheel_motor = auto()
    axle_motor = auto()

class SwerveModule():
    
    def __init__(self, module_config) -> None:
        
        self.wheel_joint_name = module_config["wheel_joint_name"]
        self.axle_joint_name = module_config["axle_joint_name"]
        self.wheel_joint_port = module_config["wheel_motor_port"]
        self.axle_joint_port = module_config["axle_motor_port"]
        self.axle_encoder_port = module_config["axle_encoder_port"]

        self.wheel_motor = ctre.TalonFX(self.wheel_joint_port)
        self.axle_motor = ctre.TalonFX(self.axle_joint_port)
        self.encoder = ctre.CANCoder(self.axle_encoder_port)

        self.last_wheel_vel_cmd = None
        self.last_axle_vel_cmd = None
        
        # Configure Encoder
        self.encoderconfig = ctre.CANCoderConfiguration()
        self.encoderconfig.sensorCoefficient = 2 * math.pi / 4096.0
        self.encoderconfig.unitString = "rad"
        self.encoderconfig.sensorTimeBase = ctre.SensorTimeBase.PerSecond
        self.encoder.configAllSettings(self.encoderconfig)
        
        # Configure Motors
        self.kf = 1023.0/20660.0
        self.kp = 0.1
        self.ki = 0.001
        self.kd = 5

        self.wheel_motor.configFactoryDefault()
        self.wheel_motor.configNeutralDeadband(0.001)
        self.wheel_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        self.wheel_motor.configNominalOutputForward(0, 30)
        self.wheel_motor.configNominalOutputReverse(0, 30)
        self.wheel_motor.configPeakOutputForward(1, 30)
        self.wheel_motor.configPeakOutputReverse(-1, 30)
        self.wheel_motor.config_kF(0, self.kf, 30)
        self.wheel_motor.config_kP(0, self.kp, 30)
        self.wheel_motor.config_kI(0, self.ki, 30)
        self.wheel_motor.config_kD(0, self.kd, 30)

        self.axle_motor.configFactoryDefault()
        self.axle_motor.configNeutralDeadband(0.001)
        self.axle_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        self.axle_motor.configNominalOutputForward(0, 30)
        self.axle_motor.configNominalOutputReverse(0, 30)
        self.axle_motor.configPeakOutputForward(1, 30)
        self.axle_motor.configPeakOutputReverse(-1, 30)
        self.axle_motor.config_kF(0, self.kf, 30)
        self.axle_motor.config_kP(0, self.kp, 30)
        self.axle_motor.config_kI(0, self.ki, 30)
        self.axle_motor.config_kD(0, self.kd, 30)


    def convertToTicks(self, angular_vel):
        return TICKS_PER_RAD * angular_vel / 10.0
 
    def scaleAxleToShaft(self, angular_vel):
        return angular_vel * AXLE_JOINT_GEAR_RATIO

    def scaleWheelToShaft(self, angular_vel):
        return angular_vel * WHEEL_JOINT_GEAR_RATIO

    def setVelocities(self, wheel_motor_vel, axle_motor_vel):
        deadzone = 0.2
        # if wheel_motor_vel < deadzone:
            # self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, 0)
        # else:
        wheel_vel = self.convertToTicks(self.scaleWheelToShaft(wheel_motor_vel - axle_motor_vel/1.9))
        self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, wheel_vel)
        self.last_wheel_vel_cmd = wheel_vel
        
        if axle_motor_vel < deadzone:
            self.axle_motor.set(ctre.TalonFXControlMode.Velocity, 0)
        else:
            axle_vel = self.convertToTicks(self.scaleAxleToShaft(axle_motor_vel))
            self.axle_motor.set(ctre.TalonFXControlMode.Velocity, axle_vel)
            self.last_axle_vel_cmd = axle_vel


    def stop(self):
        self.wheel_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.axle_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def convertToRadiansSec(self, ticks):
        return ticks * 10 / TICKS_PER_RAD
    
    def convertToRadians(self, ticks):
        return ticks / TICKS_PER_RAD
    
    def scaleShaftToAxle(self, angular_vel):
        return angular_vel * AXLE_JOINT_GEAR_RATIO

    def scaleShaftToWheel(self, angular_vel):
        return angular_vel * WHEEL_JOINT_GEAR_RATIO

    def convertEncoder(self, encoder_value) -> int:
        pi_range = abs(math.fmod(encoder_value, 4 * math.pi))
        center = pi_range - (2 * math.pi)
        scaled = int(center * 10000)
        return scaled

    def getEncoderData(self):
            output = [
                {
                    "name": self.wheel_joint_name,
                    "position": 0.0, #self.wheel_motor.getSensorCollection().getIntegratedSensorPosition() % Constants.TICKS_PER_REV, 
                    "velocity": 0.0 #self.wheel_motor.getSensorCollection().getIntegratedSensorVelocity()
                },
                {
                    "name": self.axle_joint_name,
                    "position": self.convertEncoder(self.encoder.getPosition()),
                    "velocity": 0.0 #self.encoder.getVelocity()
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
                    axle_velocity = commands['velocity'][i]

                    wheel_name = axle_name.replace('axle', 'wheel')
                    wheel_index = commands['name'].index(wheel_name)
                    wheel_velocity = commands['velocity'][wheel_index]

                    module = self.module_lookup[axle_name]
                    module.setVelocities(wheel_velocity, axle_velocity)
                    logging.info(f"{wheel_name}: {wheel_velocity}\n{axle_name}: {axle_velocity}")
        else:
            current_time = time.time()
            if current_time - self.last_cmds_time > CMD_TIMEOUT_SECONDS:
                self.stop()
                # Display Warning Once
                if self.warn_timeout:
                    logging.warn("CMD TIMEOUT: HALTING")
                    self.warn_timeout = False

