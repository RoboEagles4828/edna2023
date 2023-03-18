import logging
import wpilib
import threading
import traceback
import time
import os, inspect
from hardware_interface.drivetrain import DriveTrain
from hardware_interface.joystick import Joystick
from hardware_interface.armcontroller import ArmController
from dds.dds import DDS_Publisher, DDS_Subscriber

EMABLE_ENCODER = True
ENABLE_JOY = True
ENABLE_DRIVE =  True
ENABLE_ARM = True
ENABLE_STAGE_BROADCASTER = True
FRC_STAGE = "DISABLED"
FMS_ATTACHED = False
STOP_THREADS = False

# Locks
rti_init_lock = threading.Lock()
drive_train_lock = threading.Lock()
arm_controller_lock = threading.Lock()

# Logging
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")

# XML Path for DDS configuration
curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
xml_path = os.path.join(curr_path, "dds/xml/ROS_RTI.xml")



############################################
## Hardware
drive_train : DriveTrain = None
def initDriveTrain():
    global drive_train
    if drive_train == None:
        drive_train = DriveTrain()
        logging.info("Success: DriveTrain created")
    return drive_train

joystick : Joystick = None
def initJoystick():
    global joystick
    if joystick == None:
        joystick = Joystick()
        logging.info("Success: Joystick created")
    return joystick

arm_controller : ArmController = None
def initArmController():
    global arm_controller
    if arm_controller == None:
        arm_controller = ArmController()
        logging.info("Success: ArmController created")
    return arm_controller

############################################



############################################
## Generic Loop that is used for all threads
def threadLoop(name, dds, action):
    logging.info(f"Starting {name} thread")
    global STOP_THREADS
    global FRC_STAGE
    try:
        while STOP_THREADS == False:
            if FRC_STAGE in ["TELEOP", "AUTON"] or name in ["encoder", "stage-broadcaster"]:
                action(dds)
            time.sleep(20/1000)
    except Exception as e:
        logging.error(f"An issue occured with the {name} thread")
        logging.error(e)
        logging.error(traceback.format_exc())
    
    logging.info(f"Closing {name} thread")
    dds.close()


# Generic Start Thread Function
def startThread(name) -> threading.Thread | None:
    thread = None
    if name == "encoder":
        thread = threading.Thread(target=encoderThread, daemon=True)
    elif name == "command":
        thread = threading.Thread(target=commandThread, daemon=True)
    elif name == "arm-command":
        thread = threading.Thread(target=armThread, daemon=True)
    elif name == "joystick":
        thread = threading.Thread(target=joystickThread, daemon=True)
    elif name == "stage-broadcaster":
        thread = threading.Thread(target=stageBroadcasterThread, daemon=True)
    
    thread.start()
    return thread

############################################




######### Encoder Thread and Action #########
ENCODER_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::encoder_info"
ENCODER_WRITER_NAME = "encoder_info_publisher::encoder_info_writer"

def encoderThread():
    encoder_publisher = None
    with rti_init_lock:
        encoder_publisher = DDS_Publisher(xml_path, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
    threadLoop('encoder', encoder_publisher, encoderAction)

def encoderAction(publisher):
    # TODO: Make these some sort of null value to identify lost data
    data = {
        'name': [],
        'position': [],
        'velocity': []
    }

    global drive_train
    with drive_train_lock:
        if ENABLE_DRIVE:
            drive_data = drive_train.getEncoderData()
            data['name'] += drive_data['name']
            data['position'] += drive_data['position']
            data['velocity'] += drive_data['velocity']
    
    global arm_controller
    with arm_controller_lock:
        if ENABLE_ARM:
            arm_data = arm_controller.getEncoderData()
            data['name'] += arm_data['name']
            data['position'] += arm_data['position']
            data['velocity'] += arm_data['velocity']

    publisher.write(data)




######### Command Thread and Action #########
COMMAND_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::joint_commands"
COMMAND_WRITER_NAME = "isaac_joint_commands_subscriber::joint_commands_reader"

def commandThread():
    command_subscriber = None
    with rti_init_lock:
        command_subscriber = DDS_Subscriber(xml_path, COMMAND_PARTICIPANT_NAME, COMMAND_WRITER_NAME)
    threadLoop('command', command_subscriber, commandAction)

def commandAction(subscriber : DDS_Subscriber):
    data = subscriber.read()
    global drive_train
    with drive_train_lock:
        drive_train.sendCommands(data)




######### Arm Thread and Action #########
ARM_COMMAND_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::arm_commands"
ARM_COMMAND_WRITER_NAME = "isaac_arm_commands_subscriber::arm_commands_reader"

def armThread():
    arm_command_subscriber = None
    with rti_init_lock:
        arm_command_subscriber = DDS_Subscriber(xml_path, ARM_COMMAND_PARTICIPANT_NAME, ARM_COMMAND_WRITER_NAME)
    threadLoop('arm', arm_command_subscriber, armAction)

def armAction(subscriber : DDS_Subscriber):
    data = subscriber.read()
    global arm_controller
    with arm_controller_lock:
        arm_controller.sendCommands(data)




######### Joystick Thread and Action #########
JOYSTICK_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::joystick"
JOYSTICK_WRITER_NAME = "joystick_data_publisher::joystick_data_writer"

def joystickThread():
    joystick_publisher = None
    with rti_init_lock:
        joystick_publisher = DDS_Publisher(xml_path, JOYSTICK_PARTICIPANT_NAME, JOYSTICK_WRITER_NAME)
    threadLoop('joystick', joystick_publisher, joystickAction)

def joystickAction(publisher : DDS_Publisher):
    if FRC_STAGE == "TELEOP":
        global joystick
        data = None
        try:
            data = joystick.getData()
        except:
            logging.warn("No joystick data could be fetched!")
            initJoystick()
        publisher.write(data)




######### STAGE BROADCASTER THREAD and ACTION #########
STAGE_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::stage_broadcaster"
STAGE_WRITER_NAME = "stage_publisher::stage_writer"

def stageBroadcasterThread():
    stage_publisher = None
    with rti_init_lock:
        stage_publisher = DDS_Publisher(xml_path, STAGE_PARTICIPANT_NAME, STAGE_WRITER_NAME)
    threadLoop('stage-broadcaster', stage_publisher, stageBroadcasterAction)

def stageBroadcasterAction(publisher : DDS_Publisher):
    global FRC_STAGE
    global FMS_ATTACHED
    publisher.write({ "data": f"{FRC_STAGE}|{FMS_ATTACHED}" })





######### Robot Class #########
class EdnaRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.use_threading = True
        wpilib.CameraServer.launch()
        logging.warning("Running in simulation!") if wpilib.RobotBase.isSimulation() else logging.info("Running in real!")

        self.drive_train = initDriveTrain()
        self.joystick = initJoystick()
        self.arm_controller = initArmController()

        self.threads = []
        if self.use_threading:
            logging.info("Initializing Threads")
            global STOP_THREADS
            STOP_THREADS = False
            if ENABLE_DRIVE: self.threads.append({"name": "command", "thread": startThread("command") })
            if ENABLE_ARM: self.threads.append({"name": "arm-command", "thread": startThread("arm-command")})
            if ENABLE_JOY: self.threads.append({"name": "joystick", "thread": startThread("joystick") })
            if EMABLE_ENCODER: self.threads.append({"name": "encoder", "thread": startThread("encoder") })
            if ENABLE_STAGE_BROADCASTER: self.threads.append({"name": "stage-broadcaster", "thread": startThread("stage-broadcaster") })

        else:
            self.encoder_publisher = DDS_Publisher(xml_path, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
            self.joystick_publisher = DDS_Publisher(xml_path, JOYSTICK_PARTICIPANT_NAME, JOYSTICK_WRITER_NAME)
            self.command_subscriber = DDS_Subscriber(xml_path, COMMAND_PARTICIPANT_NAME, COMMAND_WRITER_NAME)
            self.arm_command_subscriber = DDS_Subscriber(xml_path, ARM_COMMAND_PARTICIPANT_NAME, ARM_COMMAND_WRITER_NAME)
            self.stage_publisher = DDS_Publisher(xml_path, STAGE_PARTICIPANT_NAME, STAGE_WRITER_NAME)


    # Auton
    def autonomousInit(self):
        logging.info("Entering Auton")
        global FRC_STAGE
        FRC_STAGE = "AUTON"


    def autonomousPeriodic(self):
        global FMS_ATTACHED
        FMS_ATTACHED = wpilib.DriverStation.isFMSAttached()
        if self.use_threading:
            self.manageThreads()
        else:
            self.doActions()

    def autonomousExit(self):
        logging.info("Exiting Auton")
        global FRC_STAGE
        FRC_STAGE = "AUTON"


    # Teleop
    def teleopInit(self):
        logging.info("Entering Teleop")
        global FRC_STAGE
        FRC_STAGE = "TELEOP"
    
    def teleopPeriodic(self):
        global FMS_ATTACHED
        FMS_ATTACHED = wpilib.DriverStation.isFMSAttached()
        if self.use_threading:
            self.manageThreads()
        else:
            self.doActions()

    def teleopExit(self):
        logging.info("Exiting Teleop")
        global FRC_STAGE
        FRC_STAGE = "DISABLED"
        with drive_train_lock:
            drive_train.stop()
        with arm_controller_lock:
            arm_controller.stop()

    def manageThreads(self):
        # Check all threads and make sure they are alive
        for thread in self.threads:
            if thread["thread"].is_alive() == False:
                # If this is the command thread, we need to stop the robot
                if thread["name"] == "command":
                    logging.warning(f"Stopping robot due to command thread failure")
                    with drive_train_lock:
                        drive_train.stop()
                    with arm_controller_lock:
                        arm_controller.stop()
                logging.warning(f"Thread {thread['name']} is not alive, restarting...")
                thread["thread"] = startThread(thread["name"])
    
    def doActions(self):
        encoderAction(self.encoder_publisher)
        commandAction(self.command_subscriber)
        armAction(self.arm_command_subscriber)
        joystickAction(self.joystick_publisher)
        stageBroadcasterAction(self.stage_publisher)

    # Is this needed?
    def stopThreads(self):
        global STOP_THREADS
        STOP_THREADS = True
        for thread in self.threads:
            thread.join()
        logging.info('All Threads Stopped')


if __name__ == '__main__':
    wpilib.run(EdnaRobot)
