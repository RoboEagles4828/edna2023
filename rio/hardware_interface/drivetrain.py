import wpilib
import ctre
from enum import Enum, auto
import math
   

MODULE_CONFIG = {
    "front_left": {
        "wheel_joint_name": "front_left_wheel_joint",
        "wheel_motor_port": 9,
        "axle_joint_name": "front_left_axle_joint",
        "axle_motor_port": 7,
        "axle_encoder_port": 8
    },
    "front_right": {
        "wheel_joint_name": "front_left_wheel_joint",
        "wheel_motor_port": 12,
        "axle_joint_name": "front_left_axle_joint",
        "axle_motor_port": 10,
        "axle_encoder_port": 11
    },
    "back_left": {
        "wheel_joint_name": "front_left_wheel_joint",
        "wheel_motor_port": 6,
        "axle_joint_name": "front_left_axle_joint",
        "axle_motor_port": 4,
        "axle_encoder_port": 5
    },
    "back_right": {
        "wheel_joint_name": "front_left_wheel_joint",
        "wheel_motor_port": 3,
        "axle_joint_name": "front_left_axle_joint",
        "axle_motor_port": 1,
        "axle_encoder_port": 2
    }
}
WHEEL_JOINT_GEAR_RATIO = 6.75 #8.14
AXLE_JOINT_GEAR_RATIO = 150.0/7.0
TICKS_PER_REV = 2048
TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)


class MotorType(Enum):
    wheel_motor = auto()
    axle_motor = auto()

class SwerveModule():
    
    def __init__(self, axle_joint_name, axle_joint_port, wheel_joint_name, wheel_joint_port, encoder_port) -> None:
        self.wheel_motor = ctre.TalonFX(wheel_joint_port)
        self.axle_motor = ctre.TalonFX(axle_joint_port)
        self.wheel_joint_name = wheel_joint_name
        self.axle_joint_name = axle_joint_name
        self.wheel_joint_port = wheel_joint_port
        self.axle_joint_port = axle_joint_port

        self.encoder = ctre.CANCoder(encoder_port)
        # encoderconfig = ctre.CANCoderConfiguration()
        # encoderconfig.sensorCoefficient = 2 * math.pi / 4096.0
        # encoderconfig.unitString = "rad"
        # encoderconfig.sensorTimeBase = ctre.SensorTimeBase.PerSecond
        # self.encoder.configAllSettings(encoderconfig)
        

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
        scaled_vel = Constants.TICKS_PER_RAD * angular_vel / 10.0

        return scaled_vel
 
    def scaleAxleToShaft(self, ticks_per_100ms):
        scaled_vel = ticks_per_100ms * Constants.AXLE_JOINT_GEAR_RATIO
        return scaled_vel

    def scaleWheelToShaft(self, ticks_per_100ms):
        scaled_vel = ticks_per_100ms * Constants.WHEEL_JOINT_GEAR_RATIO
        return scaled_vel

    def setVelocities(self, wheel_motor_vel, axle_motor_vel):
        print(f'WHEEL_NAME - {self.wheel_joint_name}, WHEEL_PORT - {self.wheel_joint_port}')
        print(f'SETTING VELOCITIES: WHEEL - {wheel_motor_vel}, AXLE - {axle_motor_vel}')
        wheel_vel = self.convertToTicks(self.scaleWheelToShaft(wheel_motor_vel - axle_motor_vel/1.9)) #(2-(wheel_motor_vel/(66*math.pi)))
        axle_vel = self.convertToTicks(self.scaleAxleToShaft(axle_motor_vel))
        self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, wheel_vel)
        self.axle_motor.set(ctre.TalonFXControlMode.Velocity, axle_vel)

    def stop(self):
        self.wheel_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.axle_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def convertToRadiansSec(self, ticks):
        return ticks * 10 / Constants.TICKS_PER_RAD
    
    def convertToRadians(self, ticks):
        return ticks / Constants.TICKS_PER_RAD
    
    def scaleShaftToAxle(self, ticks_per_100ms):
        scaled_vel = ticks_per_100ms * Constants.AXLE_JOINT_GEAR_RATIO
        return scaled_vel

    def scaleShaftToWheel(self, ticks_per_100ms):
        scaled_vel = ticks_per_100ms * Constants.WHEEL_JOINT_GEAR_RATI
        return scaled_vel
    

    def getEncoderInfo(self):
            # if(self.axle_joint_name == "front_right_axle_joint"):
                # print(f"{self.axle_joint_name} ENCODER VALUE: {str((math.fmod(self.encoder.getPosition(), 2 * math.pi) * -1) - math.pi)}")
            output = \
            {
                "wheel_joint": 
                    {
                        "name": self.wheel_joint_name,
                        "position": 0.0, #self.wheel_motor.getSensorCollection().getIntegratedSensorPosition() % Constants.TICKS_PER_REV, 
                        "velocity": 0.0 #self.wheel_motor.getSensorCollection().getIntegratedSensorVelocity()
                    },
                "axle_joint":
                    {
                        "name": self.axle_joint_name,
                        "position": int(((math.fmod(self.encoder.getPosition(), 2 * math.pi) * -1) - math.pi) * 10000),
                        "velocity": 0.0 #self.encoder.getVelocity()
                    }
            }
            return output
            # return {"position": self.axle_motor.getSensorCollection().getIntegratedSensorPosition(), "velocity": self.axle_motor.getSensorCollection().getIntegratedSensorVelocity()}

class DriveTrain():
    def __init__(self):
        self.front_left = SwerveModule("front_left_axle_joint", Constants.front_left_axle_motor_port, "front_left_wheel_joint", Constants.front_left_wheel_motor_port, Constants.front_left_encoder_port)
        self.front_right = SwerveModule("front_right_axle_joint", Constants.front_right_axle_motor_port, "front_right_wheel_joint", Constants.front_right_wheel_motor_port, Constants.front_right_encoder_port)
        self.back_left = SwerveModule("rear_left_axle_joint", Constants.back_left_axle_motor_port, "rear_left_wheel_joint", Constants.back_left_wheel_motor_port, Constants.back_left_encoder_port)
        self.back_right = SwerveModule("rear_right_axle_joint", Constants.back_right_axle_motor_port, "rear_right_wheel_joint", Constants.back_right_wheel_motor_port, Constants.back_right_encoder_port)
        self.module_lookup = \
        {
            'front_left_axle_joint': self.front_left,
            'front_right_axle_joint': self.front_right,
            'rear_left_axle_joint': self.back_left,
            'rear_right_axle_joint': self.back_right,

        }

    def getEncoderInfo(self):
        output = \
        {
            'front_left': self.front_left.getEncoderInfo(),
            'front_right': self.front_right.getEncoderInfo(),
            'back_left': self.back_left.getEncoderInfo(),
            'back_right': self.back_right.getEncoderInfo()
        }
        return output

    def stop(self):
        self.front_left.stop()
        self.front_right.stop()
        self.back_left.stop()
        self.back_right.stop()


    def setDynamicVelocities(self, commands):
        if commands:
            for command in commands:
                module_axle_name = command['axle_joint']['name']
                module = self.module_lookup[module_axle_name]
                module.setVelocities(command['wheel_joint']['velocity'], command['axle_joint']['velocity'])

