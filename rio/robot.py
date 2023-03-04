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

# Locks
FRC_STAGE = "DISABLED"
STOP_THREADS = False
rti_init_lock = threading.Lock()
drive_train_lock = threading.Lock()
arm_controller_lock = threading.Lock()

# Logging
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")

# XML Path for DDS configuration
curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
xml_path = os.path.join(curr_path, "dds/xml/ROS_RTI.xml")

## Hardware
drive_train : DriveTrain = None
def initDriveTrain():
    global drive_train
    drive_train = DriveTrain()
    logging.info("Success: DriveTrain created")

joystick : Joystick = None
def initJoystick():
    try:
        global joystick
        joystick = Joystick()
        logging.info("Success: Joystick created")
        return True
    except Exception as e:
        logging.error("Failed to create joystick")
        return False

arm_controller : ArmController = None
def initArmController():
    global arm_controller
    arm_controller = ArmController()
    logging.info("Success: ArmController created")

## Generic Loop that is used for all threads
def threadLoop(name, dds, action):
    logging.info(f"Starting {name} thread")
    global STOP_THREADS
    global FRC_STAGE
    try:
        while STOP_THREADS == False:
            if FRC_STAGE == "TELEOP" or FRC_STAGE == "AUTON" or name == "encoder":
                action(dds)
            time.sleep(20/1000)
    except Exception as e:
        logging.error(f"An issue occured with the {name} thread")
        logging.error(e)
        logging.error(traceback.format_exc())
    
    logging.info(f"Closing {name} thread")
    dds.close()
# Generic Start Thread Function
def startThread(name):
    thread = None
    if name == "encoder":
        thread = threading.Thread(target=encoderThread, daemon=True)
    elif name == "command":
        thread = threading.Thread(target=commandThread, daemon=True)
    elif name == "arm-command":
        thread = threading.Thread(target=armThread, daemon=True)
    elif name == "joystick":
        thread = threading.Thread(target=joystickThread, daemon=True)
    
    thread.start()
    return thread




######### Encoder Thread and Action #########
ENCODER_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::encoder_info"
ENCODER_WRITER_NAME = "encoder_info_publisher::encoder_info_writer"

def encoderThread():
    encoder_publisher = None
    with rti_init_lock:
        encoder_publisher = DDS_Publisher(xml_path, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
    threadLoop('encoder', encoder_publisher, encoderAction)

def encoderAction(publisher):
    data = None

    drive_data = None
    global drive_train
    with drive_train_lock:
        drive_data = drive_train.getEncoderData()
    
    arm_data = None
    global arm_controller
    with arm_controller_lock:
        arm_data = arm_controller.getEncoderData()

    data = {
        "name": drive_data["name"] + arm_data["name"],
        "position": drive_data["position"] + arm_data["position"],
        "velocity": drive_data["velocity"] + arm_data["velocity"]
    }
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
    global joystick
    data = None
    try:
        data = joystick.getData()
    except:
        logging.warn("No joystick data could be fetched!")
        initJoystick()
    publisher.write(data)


######### Robot Class #########
class edna_robot(wpilib.TimedRobot):

    def __init__(self, period = 0.2, use_threading = True) -> None:
        super().__init__(period)
        self.use_threading = use_threading

    def robotInit(self) -> None:
        initDriveTrain()
        # initJoystick()
        initArmController()

        self.threads = []
        if self.use_threading:
            logging.info("Initializing Threads")
            global STOP_THREADS
            STOP_THREADS = False
            self.threads = [
                {"name": "encoder", "thread": startThread("encoder") },
                {"name": "command", "thread": startThread("command") },
                {"name": "arm-command", "thread": startThread("arm-command")},
                # {"name": "joystick", "thread": startThread("joystick") },
            ]
        else:
            self.encoder_publisher = DDS_Publisher(xml_path, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
            self.joystick_publisher = DDS_Publisher(xml_path, JOYSTICK_PARTICIPANT_NAME, JOYSTICK_WRITER_NAME)
            self.command_subscriber = DDS_Subscriber(xml_path, COMMAND_PARTICIPANT_NAME, COMMAND_WRITER_NAME)
            self.arm_command_subscriber = DDS_Subscriber(xml_path, ARM_COMMAND_PARTICIPANT_NAME, ARM_COMMAND_WRITER_NAME)

    def teleopInit(self) -> None:
        logging.info("Entering Teleop")
        global FRC_STAGE
        FRC_STAGE = "TELEOP"
    
    def teleopPeriodic(self) -> None:
        if self.use_threading:
            self.manageThreads()
        else:
            self.doActions()

    def teleopExit(self) -> None:
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
        return

    def disabledInit(self) -> None:
        return
    
    # Is this needed?
    def stopThreads(self):
        global STOP_THREADS
        STOP_THREADS = True
        for thread in self.threads:
            thread.join()
        logging.info('All Threads Stopped')


if __name__ == '__main__':
    wpilib.run(edna_robot)

