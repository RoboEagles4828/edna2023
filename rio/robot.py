import logging
import wpilib
import threading
import traceback
import time
import os, inspect
from hardware_interface.drivetrain import DriveTrain
from dds.dds import DDS_Publisher, DDS_Subscriber

# Locks
FRC_STAGE = "DISABLED"
STOP_THREADS = False
rti_init_lock = threading.Lock()
drive_train_lock = threading.Lock()

# Logging
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")

# XML Path for DDS configuration
curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
xml_path = os.path.join(curr_path, "dds/xml/ROS_RTI.xml")


## Drive Train
drive_train = None
def initDriveTrain():
    global drive_train
    drive_train = DriveTrain()

## Generic Loop that is used for all threads
def threadLoop(name, dds, action):
    logging.info(f"Starting {name} thread")
    global STOP_THREADS
    global FRC_STAGE
    try:
        while STOP_THREADS == False:
            if FRC_STAGE == "TELEOP" or FRC_STAGE == "AUTON":
                action(dds)
            time.sleep(20/1000)
    except Exception as e:
        logging.error(f"An issue occured with the {name} thread")
        logging.error(e)
        logging.error(traceback.format_exc())
    
    logging.info(f"Closing {name} thread")
    dds.close()


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
    global drive_train
    with drive_train_lock:
        data = drive_train.getEncoderData()
    publisher.write(data)

######### Command Thread and Action #########
COMMAND_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::joint_commands"
COMMAND_WRITER_NAME = "isaac_joint_commands_subscriber::joint_commands_reader"

def commandThread():
    command_subscriber = None
    with rti_init_lock:
        command_subscriber = DDS_Subscriber(xml_path, COMMAND_PARTICIPANT_NAME, COMMAND_WRITER_NAME)
    threadLoop('command', command_subscriber, commandAction)

def commandAction(subscriber):
    data = subscriber.read()
    with drive_train_lock:
        drive_train.sendCommands(data)


######### Joystick Thread and Action #########
JOYSTICK_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::joystick"
JOYSTICK_WRITER_NAME = "joystick_data_publisher::joystick_data_writer"

def joystickThread():
    joystick_publisher = None
    with rti_init_lock:
        joystick_publisher = DDS_Publisher(xml_path, JOYSTICK_PARTICIPANT_NAME, JOYSTICK_WRITER_NAME)
    threadLoop('joystick', joystick_publisher, joystickAction)

def joystickAction(publisher):
    data = controller.getJoystickData()
    publisher.write(data)



######### Robot Class #########
class edna_robot(wpilib.TimedRobot):

    def __init__(self, period = 0.2, use_threading = True) -> None:
        super().__init__(period)
        self.use_threading = use_threading

    def robotInit(self) -> None:
        initDriveTrain()
        self.threads = []
        if self.use_threading:
            logging.info("Initializing Threads")
            global STOP_THREADS
            STOP_THREADS = False
            self._encoder_thread = threading.Thread(target=encoderThread, daemon=True).start()
            self._command_thread = threading.Thread(target=commandThread, daemon=True).start()
            # joystickThread = threading.Thread(target=joystickThread, daemon=True)
            self.threads = [self._encoder_thread, self._command_thread]
        else:
            self.encoder_publisher = DDS_Publisher(xml_path, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
            # self.joystick_publisher = DDS_Publisher(xml_path, JOYSTICK_PARTICIPANT_NAME, JOYSTICK_WRITER_NAME)
            self.command_subscriber = DDS_Subscriber(xml_path, COMMAND_PARTICIPANT_NAME, COMMAND_WRITER_NAME)

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
        drive_train.stop()
        # if self.use_threading:
        #     self.stopThreads()

    def manageThreads(self):
        pass
    
    def doActions(self):
        encoderAction(self.encoder_publisher)
        commandAction(self.command_subscriber)
        # joystickAction(self.joystick_publisher)
        return

    def stopThreads(self):
        global STOP_THREADS
        STOP_THREADS = True
        for thread in self.threads:
            thread.join()
        logging.info('All Threads Stopped')


if __name__ == '__main__':
    wpilib.run(edna_robot)
