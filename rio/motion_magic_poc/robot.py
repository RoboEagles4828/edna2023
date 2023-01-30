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

# Motor Constants
DRIVE_RATIO = 6.75
STEER_RATIO = 150.0/7.0
motor_ticks_per_rev = 2048.0
nominal_voltage = 12.0
steer_current_limit = 20.0

# Encoder Constants
encoder_ticks_per_rev = 4096.0
encoder_offset = 0
encoder_direction = False
encoder_reset_velocity = math.radians(0.5)
encoder_reset_iterations = 500

# Demo Behavior
# Range: 0 - 2pi
velocityConstant = 1.0
accelerationConstant = 0.5
increment = 1

# Conversion Functions
# This is thought of as radians per shaft tick times the ratio to the axle (steer)
positionCoefficient = 2.0 * math.pi / motor_ticks_per_rev / STEER_RATIO
# This is the same conversion with time in mind.
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

######### Robot Class #########
class motor_poc(wpilib.TimedRobot):

    def robotInit(self) -> None:
        logging.info("Entering Robot Init")
        # Configure Joystick
        self.joystick = wpilib.XboxController(controllerPort)

        # Configure CANCoder
        canCoderConfig = ctre.CANCoderConfiguration()
        canCoderConfig.absoluteSensorRange = ctre.AbsoluteSensorRange.Unsigned_0_to_360
        canCoderConfig.initializationStrategy = ctre.SensorInitializationStrategy.BootToAbsolutePosition
        canCoderConfig.magnetOffsetDegrees = encoder_offset
        canCoderConfig.sensorDirection = encoder_direction
        # Setup encoder to be in radians
        canCoderConfig.sensorCoefficient = 2 * math.pi / encoder_ticks_per_rev
        canCoderConfig.unitString = "rad"
        canCoderConfig.sensorTimeBase = ctre.SensorTimeBase.PerSecond
        self.encoder = ctre.CANCoder(encoderPort)
        self.encoder.configAllSettings(canCoderConfig, timeout_ms)
        self.encoder.setPositionToAbsolute(timeout_ms)
        self.encoder.setStatusFramePeriod(ctre.CANCoderStatusFrame.SensorData, 10, timeout_ms)

        # Configure Talon
        self.talon = ctre.TalonFX(motorPort)
        self.talon.configFactoryDefault()
        self.talon.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, pid_loop_idx, timeout_ms)
        self.talon.configNeutralDeadband(0.001, timeout_ms)
        self.talon.setSensorPhase(True)
        self.talon.setInverted(True)
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
        # TODO: Figure out Magic Numbers and Calculations Below
        self.talon.config_kF(slot_idx, (1023.0 *  velocityCoefficient / nominal_voltage) * velocityConstant, timeout_ms)
        self.talon.configMotionCruiseVelocity(2.0 / velocityConstant / velocityCoefficient, timeout_ms)
        self.talon.configMotionAcceleration((8.0 - 2.0) / accelerationConstant / velocityCoefficient, timeout_ms)
        # Set Sensor Position to match Absolute Position of CANCoder
        self.talon.setSelectedSensorPosition(getShaftTicks(self.encoder.getAbsolutePosition(), "position"), pid_loop_idx, timeout_ms)
        self.talon.configVoltageCompSaturation(nominal_voltage, timeout_ms)
        currentLimit = ctre.SupplyCurrentLimitConfiguration()
        currentLimit.enable = True
        currentLimit.currentLimit = steer_current_limit
        self.talon.configSupplyCurrentLimit(currentLimit, timeout_ms)
        self.talon.enableVoltageCompensation(True)
        self.talon.setNeutralMode(ctre.NeutralMode.Brake)
        
        # Keep track of position
        self.targetPosition = 0
        self.reset_iterations = 0


    def teleopInit(self) -> None:
        logging.info("Entering Teleop")
        self.talon.setSelectedSensorPosition(getShaftTicks(self.encoder.getAbsolutePosition(), "position"))
    
    def teleopPeriodic(self) -> None:
        # Get position and velocities of the motor
        motorPosition = getAxleRadians(self.talon.getSelectedSensorPosition(), "position")
        motorVelocity = getAxleRadians(self.talon.getSelectedSensorVelocity(), "velocity")
        absolutePosition = self.encoder.getAbsolutePosition()

        # Reset
        if motorVelocity < encoder_reset_velocity:
            self.reset_iterations += 1
            if self.reset_iterations >= encoder_reset_iterations:
                self.reset_iterations = 0
                self.talon.setSelectedSensorPosition(getShaftTicks(absolutePosition, "position"))
                motorPosition = absolutePosition
        else:
            self.reset_iterations = 0

        # Increment Target Position
        if self.joystick.getAButtonPressed():
            self.targetPosition += increment
        elif self.joystick.getBButtonPressed():
            self.targetPosition -= increment
        # Move back in 0 - 2pi range in case increment kicked us out
        self.targetPosition = math.fmod(self.targetPosition, 2.0 * math.pi)
        # This correction is needed in case we got a negative remainder
        # A negative radian can be thought of as starting at 2pi and moving down abs(remainder)
        if self.targetPosition < 0.0:
            self.targetPosition += 2.0 * math.pi
        


        # Now that we have a new target position, we need to figure out how to move the motor.
        # First let's assume that we will move directly to the target position.
        newMotorPosition = self.targetPosition

        # The motor could get to the target position by moving clockwise or counterclockwise.
        # The shortest path should be the direction that is less than pi radians away from the current motor position.
        # The shortest path could loop around the circle and be less than 0 or greater than 2pi.
        # We need to get the absolute current position to determine if we need to loop around the 0 - 2pi range.
        
        # The current motor position does not stay inside the 0 - 2pi range.
        # We need the absolute position to compare with the target position.
        absoluteMotorPosition = math.fmod(motorPosition, 2.0 * math.pi)
        if absoluteMotorPosition < 0.0:
            absoluteMotorPosition += 2.0 * math.pi

        # If the target position was in the first quadrant area 
        # and absolute motor position was in the last quadrant area
        # then we need to move into the next loop around the circle.
        if self.targetPosition - absoluteMotorPosition < -math.pi:
            newMotorPosition += 2.0 * math.pi
        # If the target position was in the last quadrant area
        # and absolute motor position was in the first quadrant area
        # then we need to move into the previous loop around the circle.
        elif self.targetPosition - absoluteMotorPosition > math.pi:
            newMotorPosition -= 2.0 * math.pi

        # Last, add the current existing loops that the motor has gone through.
        newMotorPosition += motorPosition - absoluteMotorPosition

        print(f"ABS Target: {self.targetPosition}    Motor Target: {newMotorPosition}    Motor Current: {motorPosition}    ABS Current: {absolutePosition}")
        
        self.talon.set(ctre.TalonFXControlMode.MotionMagic, getShaftTicks(newMotorPosition, "position"))

    def teleopExit(self) -> None:
        logging.info("Exiting Teleop")


if __name__ == '__main__':
    wpilib.run(motor_poc)

