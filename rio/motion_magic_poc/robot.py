import logging
import wpilib
import ctre

motorPort = 7
controllerPort = 0
pid_constants = {
    "kP": 0.2,
    "kI": 0.0,
    "kD": 0.0,
    "kF": 0.2,
    "kIzone": 0,
    "kPeakOutput": 1.0
}
increment = 1000
nominal_voltage = 12.0
steer_current_limit = 20.0

######### Robot Class #########
class motor_poc(wpilib.TimedRobot):

    def robotInit(self) -> None:
        logging.info("Entering Robot Init")
        slot_idx = 0
        pid_loop_idx = 0
        timeout_ms = 30
        self.joystick = wpilib.XboxController(controllerPort)
        self.talon = ctre.TalonFX(motorPort)
        self.talon.configFactoryDefault()
        self.talon.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, pid_loop_idx, timeout_ms)
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
        self.talon.configMotionCruiseVelocity(15000, timeout_ms)
        self.talon.configMotionAcceleration(6000, timeout_ms)
        self.talon.setSelectedSensorPosition(0, pid_loop_idx, timeout_ms)
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

        if self.joystick.getAButton():
            self.targetPosition += increment
        elif self.joystick.getBButton():
            self.targetPosition -= increment

        print(f"Target: {self.targetPosition} Current Position: {currentPosition} Current Motor Output: {currentMotorOutput} Current Velocity: {currentVelocity}")
        self.talon.set(ctre.TalonFXControlMode.MotionMagic, self.targetPosition)

    def teleopExit(self) -> None:
        logging.info("Exiting Teleop")


if __name__ == '__main__':
    wpilib.run(motor_poc)

