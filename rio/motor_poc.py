import logging
import wpilib
from ctre import WPI_TalonFX, TalonFXControlMode, TalonFXFeedbackDevice, StatusFrameEnhanced

motorPort = 0
talon = WPI_TalonFX(motorPort, "rio")

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


######### Robot Class #########
class motor_poc(wpilib.TimedRobot):

    def robotInit(self) -> None:
        logging.info("Entering Robot Init")
        talon.configFactoryDefault()
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, pid_loop_idx, timeout_ms)
        talon.configNeutralDeadband(0.001, timeout_ms)
        talon.setSensorPhase(False)
        talon.setInverted(False)
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout_ms)
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout_ms)
        talon.configNominalOutputForward(0, timeout_ms)
        talon.configNominalOutputReverse(0, timeout_ms)
        talon.configPeakOutputForward(1, timeout_ms)
        talon.configPeakOutputReverse(-1, timeout_ms)
        talon.selectProfileSlot(slot_idx, pid_loop_idx)
        talon.config_kF(slot_idx, pid_constants["kF"], timeout_ms)
        talon.config_kP(slot_idx, pid_constants["kP"], timeout_ms)
        talon.config_kI(slot_idx, pid_constants["kI"], timeout_ms)
        talon.config_kD(slot_idx, pid_constants["kD"], timeout_ms)
        talon.configMotionCruiseVelocity(15000, timeout_ms)
        talon.configMotionAcceleration(6000, timeout_ms)
        talon.setSelectedSensorPosition(0, pid_loop_idx, timeout_ms)

    def teleopInit(self) -> None:
        logging.info("Entering Teleop")
    
    def teleopPeriodic(self) -> None:
        currentPosition = talon.getSelectedSensorPosition()
        currentMotorOutput = talon.getMotorOutputPercent()
        currentVelocity = talon.getSelectedSensorVelocity()

        print("Current Position: " + str(currentPosition))
        print("Current Motor Output: " + str(currentMotorOutput))
        print("Current Velocity: " + str(currentVelocity))
        talon.set(TalonFXControlMode.MotionMagic, 0.5 * 2048 * 10)
        pass

    def teleopExit(self) -> None:
        logging.info("Exiting Teleop")


if __name__ == '__main__':
    wpilib.run(motor_poc)

