import wpilib
import wpilib.simulation
import ctre
import ctre.sensors
import time
import logging

NAMESPACE = 'real'
CMD_TIMEOUT_SECONDS = 1
WHEEL_TIMEOUT_MILLISECONDS = 30 # 0 means do not use the timeout
TICKS_PER_REVOLUTION = 2048.0
TOTAL_ELEVATOR_REVOLUTIONS = 160 # UNKNOWN
TOTAL_GRIPPER_REVOLUTIONS = 2   # UNKNOWN

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
    'BOTTOM_GRIPPER': [8, 9],
    # Wheels
    'ELEVATOR': 13,
    'BOTTOM_GRIPPER_LIFT': 14
}

ELEVATOR_CONFIG = {
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
    'bottom_gripper_left_arm_joint',
    'elevator_center_joint',
    'bottom_intake_joint',
]

def getJointList():
    return JOINT_LIST

class ArmController():
    def __init__(self):
        self.last_cmds_time = time.time()
        self.last_cmds = { "name" : getJointList(), "position": [0.0]*len(getJointList()), "velocity": [0.0]*len(getJointList()) }
        self.warn_timeout = True
        self.hub = wpilib.PneumaticHub(PORTS['HUB'])
        self.compressor = self.hub.makeCompressor()

        # Even though these two are technically two pistons, we're only using one solenoid to handle both
        self.arm_roller_bar = Piston(self.hub, PORTS['ARM_ROLLER_BAR']) 
        self.top_gripper_slider = Piston(self.hub, PORTS['TOP_GRIPPER_SLIDER'])
        self.top_gripper = Piston(self.hub, PORTS['TOP_GRIPPER'])
        self.bottom_gripper = Piston(self.hub, PORTS['BOTTOM_GRIPPER'])
        self.elevator = ElevatorWheel(PORTS['ELEVATOR'])
        self.bottom_gripper_lift = IntakeWheel(PORTS['BOTTOM_GRIPPER_LIFT'])

        self.JOINT_MAP : dict[str, Piston | ElevatorWheel] = {
            # Pneumatics
            'arm_roller_bar_joint': self.arm_roller_bar,
            'top_slider_joint': self.top_gripper_slider,
            'top_gripper_left_arm_joint': self.top_gripper,
            'bottom_gripper_left_arm_joint': self.bottom_gripper,
            # Wheels
            'elevator_center_joint': self.elevator,
            'bottom_intake_joint': self.bottom_gripper_lift
        }

    def getEncoderData(self):
        names = [""]*6
        positions = [0]*6
        velocities = [0]*6

        # Iterate over the JOINT_MAP and run the get() function for each of them
        for index, joint_name in enumerate(self.JOINT_MAP.keys()):
            names[index] = joint_name
            positions[index] = int(self.JOINT_MAP[joint_name].getPosition() * SCALING_FACTOR_FIX)
            velocities[index] = int(self.JOINT_MAP[joint_name].getVelocity() * SCALING_FACTOR_FIX)
        return { "name" : names, "position": positions, "velocity": velocities}

    # TODO: Add this later!!!
    def stop(self): 0

    def sendCommands(self, commands):
        if commands:
            self.last_cmds_time = time.time()
            self.warn_timeout = True
            for i in range(len(commands["name"])):
                self.JOINT_MAP[commands["name"][i]].setPosition(commands['position'][i])
        
        elif (time.time() - self.last_cmds_time > CMD_TIMEOUT_SECONDS):
            self.stop()
            if self.warn_timeout:
                logging.warning(f"Didn't recieve any commands for {CMD_TIMEOUT_SECONDS} second(s). Halting...")
                self.warn_timeout = False

class Piston():

    def __init__(self, hub : wpilib.PneumaticHub, ports : list[int]):
        self.solenoid = hub.makeDoubleSolenoid(ports[0], ports[1])

    def getPosition(self) -> int:
        return 1 if self.solenoid.get().value == wpilib.DoubleSolenoid.Value.kForward else 0
    
    # The Solenoids don't have a velocity value, so we set it to zero here
    def getVelocity(self) -> int: return 0

    def setPosition(self, position : int):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse if position == 0 else wpilib.DoubleSolenoid.Value.kForward)


class TalonWheel(ctre.WPI_TalonFX):

    def __init__(self, port : int, totalRevolutions : int):
        super().__init__(port, "rio")
        self.totalRevolutions = totalRevolutions

        self.configFactoryDefault(WHEEL_TIMEOUT_MILLISECONDS)

        # Voltage
        self.configVoltageCompSaturation(12, WHEEL_TIMEOUT_MILLISECONDS)
        self.enableVoltageCompensation(True)
        
        # Sensors and frame
        self.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, WHEEL_TIMEOUT_MILLISECONDS)
        self.configIntegratedSensorInitializationStrategy(ctre.sensors.SensorInitializationStrategy.BootToZero)
        self.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, WHEEL_TIMEOUT_MILLISECONDS)
        self.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, WHEEL_TIMEOUT_MILLISECONDS)
        
        # Nominal and Peak
        self.configNominalOutputForward(0, WHEEL_TIMEOUT_MILLISECONDS)
        self.configNominalOutputReverse(0, WHEEL_TIMEOUT_MILLISECONDS)
        self.configPeakOutputForward(1, WHEEL_TIMEOUT_MILLISECONDS)
        self.configPeakOutputReverse(-1, WHEEL_TIMEOUT_MILLISECONDS)
    
    def getPosition(self) -> float:
        return (self.getSelectedSensorPosition() / (TICKS_PER_REVOLUTION * self.totalRevolutions))
    
    def getVelocity(self) -> float:
        return (self.getSelectedSensorVelocity() * 10) / (TICKS_PER_REVOLUTION * self.totalRevolutions)
    
    def setPosition(self, position : float): # Position should be between 0.0 and 1.0
        if wpilib.RobotBase.isSimulation():
            self.setSelectedSensorPosition(position * TICKS_PER_REVOLUTION * self.totalRevolutions)
        else:
            self.set(ctre.TalonFXControlMode.Position, position * (TICKS_PER_REVOLUTION * self.totalRevolutions))


class IntakeWheel():
    def __init__(self, port : int):
        return
        super().__init__(port, TOTAL_GRIPPER_REVOLUTIONS)

        self.setSensorPhase(False)
        self.setInverted(False)

        self.selectProfileSlot(ELEVATOR_CONFIG['SLOT'], 0)
        self.config_kP(ELEVATOR_CONFIG['SLOT'], ELEVATOR_CONFIG['kP'], WHEEL_TIMEOUT_MILLISECONDS)
        self.config_kI(ELEVATOR_CONFIG['SLOT'], ELEVATOR_CONFIG['kI'], WHEEL_TIMEOUT_MILLISECONDS)
        self.config_kD(ELEVATOR_CONFIG['SLOT'], ELEVATOR_CONFIG['kD'], WHEEL_TIMEOUT_MILLISECONDS)
        self.config_kD(ELEVATOR_CONFIG['SLOT'], ELEVATOR_CONFIG['kF'], WHEEL_TIMEOUT_MILLISECONDS)

        self.configMotionCruiseVelocity(ELEVATOR_CONFIG['MAX_SPEED'], WHEEL_TIMEOUT_MILLISECONDS) # Sets the maximum speed of motion magic (ticks/100ms)
        self.configMotionAcceleration(ELEVATOR_CONFIG['MAX_SPEED'], WHEEL_TIMEOUT_MILLISECONDS) # Sets the maximum acceleration of motion magic (ticks/100ms)

    
    def getPosition(self) -> float:
        return 0.0
    
    def getVelocity(self) -> float:
        return 0.0
    
    def setPosition(self, position : float): # Position should be between 0.0 and 1.0
        return


class ElevatorWheel(TalonWheel):
    def __init__(self, port : int):
        super().__init__(port, TOTAL_ELEVATOR_REVOLUTIONS)

        self.setSensorPhase(False)
        self.setInverted(False)

        self.selectProfileSlot(ELEVATOR_CONFIG['SLOT'], 0)
        self.config_kP(ELEVATOR_CONFIG['SLOT'], ELEVATOR_CONFIG['kP'], WHEEL_TIMEOUT_MILLISECONDS)
        self.config_kI(ELEVATOR_CONFIG['SLOT'], ELEVATOR_CONFIG['kI'], WHEEL_TIMEOUT_MILLISECONDS)
        self.config_kD(ELEVATOR_CONFIG['SLOT'], ELEVATOR_CONFIG['kD'], WHEEL_TIMEOUT_MILLISECONDS)
        self.config_kD(ELEVATOR_CONFIG['SLOT'], ELEVATOR_CONFIG['kF'], WHEEL_TIMEOUT_MILLISECONDS)

        self.configMotionCruiseVelocity(ELEVATOR_CONFIG['MAX_SPEED'], WHEEL_TIMEOUT_MILLISECONDS) # Sets the maximum speed of motion magic (ticks/100ms)
        self.configMotionAcceleration(ELEVATOR_CONFIG['MAX_SPEED'], WHEEL_TIMEOUT_MILLISECONDS) # Sets the maximum acceleration of motion magic (ticks/100ms)
    
    def getPosition(self) -> float:
        return super().getPosition() * 2
    
    def getVelocity(self) -> float:
        return super().getVelocity() * 2


    def setPosition(self, position : float): # Position should be between 0.0 and 2.0
        # print(f"Setting elevator position to {position} (converted to {(position / 2) * (TICKS_PER_REVOLUTION * TOTAL_ELEVATOR_REVOLUTIONS)})")
        if wpilib.RobotBase.isSimulation():
            self.setSelectedSensorPosition((position / 2) * (TICKS_PER_REVOLUTION * TOTAL_ELEVATOR_REVOLUTIONS))
        else:
            self.set(ctre.TalonFXControlMode.MotionMagic, (position / 2) * (TICKS_PER_REVOLUTION * TOTAL_ELEVATOR_REVOLUTIONS))