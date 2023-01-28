# from robot import edna_robot
from dds.dds import DDS_Publisher, DDS_Subscriber
import time
import os, inspect


def do_thing():
    curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    xml_path = os.path.join(curr_path, "dds/xml/ROS_RTI.xml")
    participant = "ROS2_PARTICIPANT_LIB::joint_commands"
    reader = "isaac_joint_commands_subscriber::joint_commands_reader"
    subscriber = DDS_Subscriber(xml_path, participant, reader)

    while True:
        test = subscriber.read()
        print(test)
        time.sleep(20/1000)


if __name__ == '__main__':
    do_thing()
    # robot = edna_robot()
    # robot.robotInit()
    # robot.teleopInit()
    # while True:
    #     robot.teleopPeriodic()
    #     time.sleep(20/1000)