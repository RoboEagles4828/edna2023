import logging
import wpilib
import ctre
import math

# Ports
motorPort = 7
controllerPort = 0
encoderPort = 8
# PID Constants
pid_constants = {
    "kP": 0.2,
    "kI": 0.0,
    "kD": 0.1,
    "kF": 0.2,
    "kIzone": 0,
    "kPeakOutput": 1.0
}
slot_idx = 0
pid_loop_idx = 0
timeout_ms = 30
velocityConstant = 1.0
accelerationConstant = 0.5
# Motor Constants
drive_reduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
steer_reduction = (15.0 / 32.0) * (10.0 / 60.0)
ticks_per_rotation = 2048.0
nominal_voltage = 12.0
steer_current_limit = 20.0
# Encoder Constants
encoder_ticks_per_rev = 2048
encoder_offset = 0
encoder_direction = False
# Demo Behavior
increment = 1

def get_radians_from_degrees(degrees):
    return 

######### Robot Class #########
class motor_poc(wpilib.TimedRobot):

    def robotInit(self) -> None:
        logging.info("Entering Robot Init")

        self.positionCoefficient = 2.0 * math.pi / ticks_per_rotation * steer_reduction
        self.velicityCoefficient = self.positionCoefficient * 10.0

        canCoderConfig = ctre.CANCoderConfiguration()
        canCoderConfig.absoluteSensorRange = ctre.AbsoluteSensorRange.Unsigned_0_to_360
        canCoderConfig.initializationStrategy = ctre.SensorInitializationStrategy.BootToAbsolutePosition
        canCoderConfig.magnetOffsetDegrees = encoder_offset
        canCoderConfig.sensorDirection = encoder_direction
        self.encoder = ctre.CANCoder(encoderPort)
        self.encoder.setPositionToAbsolute(timeout_ms)
        self.encoder.configAllSettings(canCoderConfig, timeout_ms)
        self.encoder.setStatusFramePeriod(ctre.CANCoderStatusFrame.SensorData, 10, timeout_ms)

        
        
        self.joystick = wpilib.XboxController(controllerPort)
        self.talon = ctre.TalonFX(motorPort)
        self.talon.configFactoryDefault()
        # self.talon.configRemoteFeedbackFilter(self.encoder, 0, timeout_ms)
        self.talon.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, pid_loop_idx, timeout_ms)
        self.talon.configNeutralDeadband(0.001, timeout_ms)
        self.talon.setSensorPhase(True)
        self.talon.setInverted(False)
        self.talon.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout_ms)
        self.talon.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout_ms)
        self.talon.configNominalOutputForward(0, timeout_ms)
        self.talon.configNominalOutputReverse(0, timeout_ms)
        self.talon.configPeakOutputForward(1, timeout_ms)
        self.talon.configPeakOutputReverse(-1, timeout_ms)
        self.talon.selectProfileSlot(slot_idx, pid_loop_idx)
        self.talon.config_kP(slot_idx, pid_constants["kP"], timeout_ms)
        self.talon.config_kI(slot_idx, pid_constants["kI"], timeout_ms)
        self.talon.config_kD(slot_idx, pid_constants["kD"], timeout_ms)
        self.talon.config_kF(slot_idx, (1023.0 *  self.velicityCoefficient / nominal_voltage) * velocityConstant, timeout_ms)
        self.talon.configMotionCruiseVelocity(2.0 / velocityConstant / self.velicityCoefficient, timeout_ms)
        self.talon.configMotionAcceleration((8.0 - 2.0) / accelerationConstant / self.velicityCoefficient, timeout_ms)
        
        self.talon.setSelectedSensorPosition(math.radians(self.encoder.getAbsolutePosition()) / self.positionCoefficient, pid_loop_idx, timeout_ms)
        self.talon.configVoltageCompSaturation(nominal_voltage, timeout_ms)
        currentLimit = ctre.SupplyCurrentLimitConfiguration()
        currentLimit.enable = True
        currentLimit.currentLimit = steer_current_limit
        self.talon.configSupplyCurrentLimit(currentLimit, timeout_ms)
        self.talon.enableVoltageCompensation(True)
        self.talon.setNeutralMode(ctre.NeutralMode.Brake)
        # Keep track of position
        self.targetPosition = 0


    def teleopInit(self) -> None:
        logging.info("Entering Teleop")
    
    def teleopPeriodic(self) -> None:
        motorPosition = self.talon.getSelectedSensorPosition() / self.positionCoefficient
        currentMotorOutput = self.talon.getMotorOutputPercent()
        currentVelocity = self.talon.getSelectedSensorVelocity()
        absolutePosition = self.encoder.getPosition()

        if self.joystick.getAButton():
            self.targetPosition += increment
        elif self.joystick.getBButton():
            self.targetPosition -= increment

        print(f"Target: {self.targetPosition} ABS POS: {absolutePosition}")
        # print(f"Target: {self.targetPosition} Current Position: {currentPosition} Current Motor Output: {currentMotorOutput} Current Velocity: {currentVelocity}")
        
        # currentPosition = math.fmod(motorPosition, 2.0 * math.pi)
        motorPostion = self.targetPosition 
        self.talon.set(ctre.TalonFXControlMode.MotionMagic, self.targetPosition / self.positionCoefficient)

    def teleopExit(self) -> None:
        logging.info("Exiting Teleop")


if __name__ == '__main__':
    wpilib.run(motor_poc)

