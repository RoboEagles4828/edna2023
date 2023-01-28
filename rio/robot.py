import logging
import wpilib
import threading
import time
import os, inspect
from hardware_interface.drivetrain import DriveTrain
from dds.dds import DDS_Publisher, DDS_Subscriber

# Locks
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
    try:
        while STOP_THREADS == False:
            action(dds)
            time.sleep(20/1000)
    except:
        logging.error(f"An issue occured with the {name} thread")
    
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
        super(period)
        self.use_threading = use_threading

    def robotInit(self) -> None:
        initDriveTrain()
        self.threads = []

    def teleopInit(self) -> None:
        if self.use_threading:
            logging.info("Initializing Threads")
            global STOP_THREADS
            STOP_THREADS = False
            encoderThread = threading.Thread(target=encoderThread, daemon=True)
            commandThread = threading.Thread(target=commandThread, daemon=True)
            joystickThread = threading.Thread(target=joystickThread, daemon=True)
            self.threads = [encoderThread, commandThread, joystickThread]
        else:
            self.encoder_publisher = DDS_Publisher(xml_path, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
            self.joystick_publisher = DDS_Publisher(xml_path, JOYSTICK_PARTICIPANT_NAME, JOYSTICK_WRITER_NAME)
            self.command_subscriber = DDS_Subscriber(xml_path, COMMAND_PARTICIPANT_NAME, COMMAND_WRITER_NAME)
    
    def teleopPeriodic(self) -> None:
        if self.use_threading:
            self.manageThreads()
        else:
            self.doActions()

    def teleopExit(self) -> None:
        logging.info("Exiting Teleop")
        if self.use_threading:
            self.stop_threads()

    def manageThreads(self):
        for thread in self.threads:
            if not thread.is_alive():
                thread.start()
    
    def doActions(self):
        encoderAction(self.encoder_publisher)
        commandAction(self.command_subscriber)
        joystickAction(self.joystick_publisher)
        return

    def stopThreads(self):
        global STOP_THREADS
        STOP_THREADS = True
        for thread in self.threads:
            thread.join()
        logging.info('All Threads Stopped')


if __name__ == '__main__':
    wpilib.run(edna_robot)




# print(self.threads)
        # for index in range(len(self.threads)):
        #     thread = self.threads[index]
        #     if thread['thread'] is None:
        #         thread['thread'] = self.start_thread(thread['name'])
        #         thread['thread'].start()
        #         time.sleep(0.5)
        #         self.threads[index] = thread
        #     else:
        #         if not thread['thread'].is_alive():
        #             thread['thread'] = self.start_thread(thread['name'])
        #             thread['thread'].start()
        #             self.threads[index] = thread
# def start_thread(self, name):
    #     print(f'STARTING THREAD -- {name}')

    #     if 'joystick' in name:
    #         return mp.Thread(target=self.joystick_thread_ptr, name='joystick', args=(self.drivetrain.controller, ))
    #     elif 'joint' in name:
    #         return mp.Thread(target=self.joint_commands_thread_ptr, name='joint_commands', args=(self.drivetrain, ))
    #     elif 'encoder' in name:
    #         return mp.Thread(target=self.encoder_info_thread_ptr, name='encoder_info', args=(self.drivetrain, ))
    #     else:
    #         print(f'THREAD -- {name} -- NOT FOUND')

# def pingWatchdog(self):
    #     watchdog = wpilib.Watchdog(20/1000)
    #     watchdog.reset()

    # def disabledInit(self) -> None:
    #     if mp.active_count() > 0:
    #         print("Killing all remaining threads")
    #         for thread in mp.enumerate():
    #             print(f'Killing {thread.name}')
    #             thread.join()
    #         print("All threads killed")

# def joystick_loop(self, controller):
#     curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#     rel_path = "dds/xml/ROS_RTI.xml"
#     xml_path = os.path.join(curr_path, rel_path)
#     config_name="ROS2_PARTICIPANT_LIB::joystick"
#     with rti.open_connector(config_name=config_name, url=xml_path) as connector:
#         joystick_writer = joystick.JoyStickWriter(connector)
#         while True:
#             if self._stop_threads is True:
#                 # joystick_writer.closeConnector()
#                 break
            
#             axes = [controller.getLeftX(), controller.getLeftY(), controller.getRightX(), controller.getRightY()]
#             buttons = [int(controller.getAButton()), int(controller.getBButton()), int(controller.getXButton()), int(controller.getYButton())]

#             joystick_writer.sendData(axes, buttons)
#             time.sleep(20/1000) #20ms teleopPeriodic loop time
        
# def joint_commands_thread_ptr(self, drivetrain: dt.DriveTrain):
#     curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#     rel_path = "dds/xml/ROS_RTI.xml"
#     xml_path = os.path.join(curr_path, rel_path)
#     config_name="ROS2_PARTICIPANT_LIB::joint_commands"
#     with rti.open_connector(config_name=config_name, url=xml_path) as connector:
#         joint_commands_reader = joint_cmds.JointCommandsReader(connector)
#         while True:
#             #TODO: fix the data format
#             data = joint_commands_reader.readData()

#             if self._stop_threads is True:
#                 # joint_commands_reader.closeConnector()
#                 break
            
#             while data is None:
#                 data = joint_commands_reader.readData()

#             if self._stop_threads is True:
#                 # joint_commands_reader.closeConnector()   
#                 break

#             if data:
#                 if data == 'HALT':
#                     print('RAMPING MOTORS DOWN')
#                     with self._lock:
#                         drivetrain.stop()
#                     continue
#                 with self._lock:
#                     # drivetrain.setTestVelocity(run_wheel_velocities[0], turn_wheel_velocities[0])
#                     drivetrain.setDynamicVelocities(data)
#     print('CLEANING UP')

# def encoder_info_thread_ptr(self, drivetrain):
    
#     config_name="ROS2_PARTICIPANT_LIB::encoder_info"
#     with rti.open_connector(config_name=config_name, url=xml_path) as connector:
#         encoder_info_writer = encoder_info.EncoderInfoWriter(connector)
#         while True:
#             info = None
#             if self._stop_threads is True:
#                 # encoder_info_writer.closeConnector()
#                 break
            
#             with self._lock:
#                 info = drivetrain.getEncoderInfo()

#             encoder_info_writer.sendData(info)
#             time.sleep(20/1000) #20ms teleopPeriodic loop time