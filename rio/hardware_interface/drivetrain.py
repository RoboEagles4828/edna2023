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
    "rear_left": {
        "wheel_joint_name": "front_left_wheel_joint",
        "wheel_motor_port": 6,
        "axle_joint_name": "front_left_axle_joint",
        "axle_motor_port": 4,
        "axle_encoder_port": 5
    },
    "rear_right": {
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
        # encoderconfig = ctre.CANCoderConfiguration()
        # encoderconfig.sensorCoefficient = 2 * math.pi / 4096.0
        # encoderconfig.unitString = "rad"
        # encoderconfig.sensorTimeBase = ctre.SensorTimeBase.PerSecond
        # self.encoder.configAllSettings(encoderconfig)
        
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
        wheel_vel = self.convertToTicks(self.scaleWheelToShaft(wheel_motor_vel - axle_motor_vel/1.9))
        axle_vel = self.convertToTicks(self.scaleAxleToShaft(axle_motor_vel))
        if wheel_vel != self.last_wheel_vel_cmd:
            self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, wheel_vel)
        if axle_vel != self.last_axle_vel_cmd:
            self.axle_motor.set(ctre.TalonFXControlMode.Velocity, axle_vel)
        self.last_wheel_vel_cmd = wheel_vel
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

    def getEncoderData(self):
            output = [
                {
                    "name": self.wheel_joint_name,
                    "position": 0.0, #self.wheel_motor.getSensorCollection().getIntegratedSensorPosition() % Constants.TICKS_PER_REV, 
                    "velocity": 0.0 #self.wheel_motor.getSensorCollection().getIntegratedSensorVelocity()
                },
                {
                    "name": self.axle_joint_name,
                    "position": int(((math.fmod(self.encoder.getPosition(), 2 * math.pi) * -1) - math.pi) * 10000),
                    "velocity": 0.0 #self.encoder.getVelocity()
                }
            ]
            return output

class DriveTrain():
    def __init__(self):
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
            for i in range(commands['name'].size):
                if 'axle' in commands['name'][i]:
                    axle_name = commands['name'][i]
                    axle_velocity = commands['velocity'][i]

                    wheel_name = axle_name.replace('axle', 'wheel')
                    wheel_index = commands['name'].index(wheel_name)
                    wheel_velocity = commands['velocity'][wheel_index]

                    module = self.module_lookup[axle_name]
                    module.setVelocities(wheel_velocity, axle_velocity)
        else:
            self.stop()

