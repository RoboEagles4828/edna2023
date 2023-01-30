import logging
import wpilib
import ctre

# Ports
motorPort = 7
controllerPort = 0
encoderPort = 0
# PID Constants
pid_constants = {
    "kP": 0.2,
    "kI": 0.0,
    "kD": 0.0,
    "kF": 0.2,
    "kIzone": 0,
    "kPeakOutput": 1.0
}
slot_idx = 0
pid_loop_idx = 0
timeout_ms = 30
# Motor Constants
nominal_voltage = 12.0
steer_current_limit = 20.0
# Encoder Constants
encoder_ticks_per_rev = 2048
encoder_offset = 0
encoder_direction = False
# Demo Behavior
increment = 10

######### Robot Class #########
class motor_poc(wpilib.TimedRobot):

    def robotInit(self) -> None:
        logging.info("Entering Robot Init")

        canCoderConfig = ctre.CANCoderConfiguration()
        canCoderConfig.absoluteSensorRange = ctre.AbsoluteSensorRange.Unsigned_0_to_360
        canCoderConfig.magnetOffsetDegrees = encoder_offset
        canCoderConfig.sensorDirection = encoder_direction
        self.encoder = ctre.CANCoder(encoderPort)
        self.encoder.configAllSettings(canCoderConfig, timeout_ms)
        self.encoder.setStatusFramePeriod(ctre.CANCoderStatusFrame.SensorData, 10, timeout_ms)
        
        self.joystick = wpilib.XboxController(controllerPort)
        self.talon = ctre.TalonFX(motorPort)
        self.talon.configFactoryDefault()
        self.talon.configRemoteFeedbackFilter(self.encoder, 0, timeout_ms)
        self.talon.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.RemoteSensor0, pid_loop_idx, timeout_ms)
        self.talon.configNeutralDeadband(0.001, timeout_ms)
        self.talon.setSensorPhase(False)
        self.talon.setInverted(False)
        self.talon.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout_ms)
        self.talon.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout_ms)
        self.talon.configNominalOutputForward(0, timeout_ms)
        self.talon.configNominalOutputReverse(0, timeout_ms)
        self.talon.configPeakOutputForward(1, timeout_ms)
        self.talon.configPeakOutputReverse(-1, timeout_ms)
        self.talon.selectProfileSlot(slot_idx, pid_loop_idx)
        self.talon.config_kF(slot_idx, pid_constants["kF"], timeout_ms)
        self.talon.config_kP(slot_idx, pid_constants["kP"], timeout_ms)
        self.talon.config_kI(slot_idx, pid_constants["kI"], timeout_ms)
        self.talon.config_kD(slot_idx, pid_constants["kD"], timeout_ms)
        self.talon.configMotionCruiseVelocity(150, timeout_ms)
        self.talon.configMotionAcceleration(60, timeout_ms)
        # self.talon.setSelectedSensorPosition(0, pid_loop_idx, timeout_ms)
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
        currentPosition = self.talon.getSelectedSensorPosition()
        currentMotorOutput = self.talon.getMotorOutputPercent()
        currentVelocity = self.talon.getSelectedSensorVelocity()
        absolutePosition = self.encoder.getAbsolutePosition()

        if self.joystick.getAButton():
            self.targetPosition += increment
        elif self.joystick.getBButton():
            self.targetPosition -= increment

        print(f"ABS POS: {absolutePosition}")
        # print(f"Target: {self.targetPosition} Current Position: {currentPosition} Current Motor Output: {currentMotorOutput} Current Velocity: {currentVelocity}")
        self.talon.set(ctre.TalonFXControlMode.MotionMagic, self.targetPosition)

    def teleopExit(self) -> None:
        logging.info("Exiting Teleop")


if __name__ == '__main__':
    wpilib.run(motor_poc)

