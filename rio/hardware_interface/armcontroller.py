import wpilib
import wpilib.simulation
import ctre
import ctre.sensors
import time
import logging

NAMESPACE = 'real'
CMD_TIMEOUT_SECONDS = 1
MOTOR_TIMEOUT = 30 # 0 means do not use the timeout
TICKS_PER_REVOLUTION = 2048.0
TOTAL_ELEVATOR_REVOLUTIONS = 160
TOTAL_INTAKE_REVOLUTIONS = 6
SCALING_FACTOR_FIX = 10000

# Port Numbers for all of the Solenoids and other connected things
# The numbers below will **need** to be changed to fit the robot wiring
PORTS = {
    # Modules
    'HUB': 18,
    # Pistons
    'ARM_ROLLER_BAR': [14, 15],
    'TOP_GRIPPER_SLIDER': [10, 11],
    'TOP_GRIPPER': [13, 12],
    # Wheels
    'ELEVATOR': 13,
    'BOTTOM_GRIPPER_LIFT': 14
}

MOTOR_PID_CONFIG = {
    'SLOT': 2,
    'MAX_SPEED': 15000,             # Ticks/100ms 
    'TARGET_ACCELERATION': 6000,    # Ticks/100ms
    "kP": 0.2,
    "kI": 0.0,
    "kD": 0.1,
    "kF": 0.2,
}

JOINT_LIST = [
    'arm_roller_bar_joint',
    'top_slider_joint',
    'top_gripper_left_arm_joint',
    'elevator_center_joint',
    'bottom_intake_joint',
]

def getJointList():
    return JOINT_LIST

class ArmController():
    def __init__(self):
        self.last_cmds_time = time.time()
        self.warn_timeout = True
        
        self.hub = wpilib.PneumaticHub(PORTS['HUB'])
        self.compressor = self.hub.makeCompressor()

        self.arm_roller_bar = Piston(self.hub, PORTS['ARM_ROLLER_BAR']) 
        self.top_gripper_slider = Piston(self.hub, PORTS['TOP_GRIPPER_SLIDER'])
        self.top_gripper = Piston(self.hub, PORTS['TOP_GRIPPER'])
        self.elevator = Elevator(PORTS['ELEVATOR'])
        self.bottom_gripper_lift = Intake(PORTS['BOTTOM_GRIPPER_LIFT'])

        self.JOINT_MAP = {
            # Pneumatics
            'arm_roller_bar_joint': self.arm_roller_bar,
            'top_slider_joint': self.top_gripper_slider,
            'top_gripper_left_arm_joint': self.top_gripper,
            # Wheels
            'elevator_center_joint': self.elevator,
            'bottom_intake_joint': self.bottom_gripper_lift
        }

    def getEncoderData(self):
        names = [""]*5
        positions = [0]*5
        velocities = [0]*5

        # Iterate over the JOINT_MAP and run the get() function for each of them
        for index, joint_name in enumerate(self.JOINT_MAP):
            names[index] = joint_name
            # Allow for joints to be removed quickly
            positions[index] = int(self.JOINT_MAP[joint_name].getPosition() * SCALING_FACTOR_FIX)
            velocities[index] = int(self.JOINT_MAP[joint_name].getVelocity() * SCALING_FACTOR_FIX)
        return { "name" : names, "position": positions, "velocity": velocities}

    def stop(self):
        for joint in self.JOINT_MAP.values():
            joint.stop()

    def sendCommands(self, commands):
        if commands:
            self.last_cmds_time = time.time()
            self.warn_timeout = True
            for i in range(len(commands["name"])):
                joint_name = commands["name"][i]
                if joint_name in self.JOINT_MAP:
                    self.JOINT_MAP[joint_name].setPosition(commands['position'][i])
        
        elif (time.time() - self.last_cmds_time > CMD_TIMEOUT_SECONDS):
            self.stop()
            if self.warn_timeout:
                logging.warning(f"Didn't recieve any commands for {CMD_TIMEOUT_SECONDS} second(s). Halting...")
                self.warn_timeout = False

class Piston():
    def __init__(self, hub : wpilib.PneumaticHub, ports : list[int]):
        self.solenoid = hub.makeDoubleSolenoid(ports[0], ports[1])
        self.state = 0

    def getPosition(self):
        return float(self.state)
    
    # The Solenoids don't have a velocity value, so we set it to zero here
    def getVelocity(self):
        return 0.0
    
    def stop(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kOff)

    def setPosition(self, position : float):
        if position >= 0.5 and self.state == 0:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
            self.state = 1
        elif position < 0.5 and self.state == 1:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
            self.state = 0

def commonTalonSetup(talon : ctre.WPI_TalonFX):
    talon.configFactoryDefault(MOTOR_TIMEOUT)
    talon.configNeutralDeadband(0.01, MOTOR_TIMEOUT)

    # Voltage
    talon.configVoltageCompSaturation(12, MOTOR_TIMEOUT)
    talon.enableVoltageCompensation(True)
    
    # Sensor
    talon.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, MOTOR_TIMEOUT)
    talon.configIntegratedSensorInitializationStrategy(ctre.sensors.SensorInitializationStrategy.BootToZero)

    # PID
    talon.selectProfileSlot(MOTOR_PID_CONFIG['SLOT'], 0)
    talon.config_kP(MOTOR_PID_CONFIG['SLOT'], MOTOR_PID_CONFIG['kP'], MOTOR_TIMEOUT)
    talon.config_kI(MOTOR_PID_CONFIG['SLOT'], MOTOR_PID_CONFIG['kI'], MOTOR_TIMEOUT)
    talon.config_kD(MOTOR_PID_CONFIG['SLOT'], MOTOR_PID_CONFIG['kD'], MOTOR_TIMEOUT)
    talon.config_kD(MOTOR_PID_CONFIG['SLOT'], MOTOR_PID_CONFIG['kF'], MOTOR_TIMEOUT)
    
    # Nominal and Peak
    talon.configNominalOutputForward(0, MOTOR_TIMEOUT)
    talon.configNominalOutputReverse(0, MOTOR_TIMEOUT)
    talon.configPeakOutputForward(1, MOTOR_TIMEOUT)
    talon.configPeakOutputReverse(-1, MOTOR_TIMEOUT)
    return


class Intake():
    def __init__(self, port : int):
        self.motor = ctre.WPI_TalonFX(port, "rio")
        commonTalonSetup(self.motor)
        self.state = 0

        # Phase
        self.motor.setSensorPhase(False)
        self.motor.setInverted(False)
        
        # Frames
        self.motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, MOTOR_TIMEOUT)

        # Brake
        self.motor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def getPosition(self):
        return (self.motor.getSelectedSensorPosition() / (-TICKS_PER_REVOLUTION * TOTAL_INTAKE_REVOLUTIONS))

    def getVelocity(self):
        return (self.motor.getSelectedSensorVelocity() * 10) / (-TICKS_PER_REVOLUTION * TOTAL_INTAKE_REVOLUTIONS)

    def stop(self):
        self.motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def setPosition(self, position : float): 
        if position >= 0.5 and self.state == 0:
            self.motor.set(ctre.TalonFXControlMode.Velocity, -TICKS_PER_REVOLUTION/2)
            self.state = 1
        elif position < 0.5 and self.state == 1:
            self.motor.set(ctre.TalonFXControlMode.Velocity, TICKS_PER_REVOLUTION/2)
            self.state = 0


class Elevator():
    def __init__(self, port : int):
        self.motor = ctre.WPI_TalonFX(port, "rio")
        commonTalonSetup(self.motor)

        # Phase
        self.motor.setSensorPhase(False)
        self.motor.setInverted(False)

        # Frames
        self.motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, MOTOR_TIMEOUT)

        # Motion Magic
        self.motor.configMotionCruiseVelocity(MOTOR_PID_CONFIG['MAX_SPEED'], MOTOR_TIMEOUT) # Sets the maximum speed of motion magic (ticks/100ms)
        self.motor.configMotionAcceleration(MOTOR_PID_CONFIG['TARGET_ACCELERATION'], MOTOR_TIMEOUT) # Sets the maximum acceleration of motion magic (ticks/100ms)
    
    def getPosition(self) -> float:
        return (self.motor.getSelectedSensorPosition() / (TICKS_PER_REVOLUTION * TOTAL_ELEVATOR_REVOLUTIONS))
    
    def getVelocity(self) -> float:
        return (self.motor.getSelectedSensorVelocity() * 10) / (TICKS_PER_REVOLUTION * TOTAL_ELEVATOR_REVOLUTIONS)

    def stop(self):
        self.motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def setPosition(self, position : float): # Position should be between 0.0 and 1.0
        self.motor.set(ctre.TalonFXControlMode.MotionMagic, position * (TICKS_PER_REVOLUTION * TOTAL_ELEVATOR_REVOLUTIONS))