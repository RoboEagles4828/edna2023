#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from threading import Thread

JOINT_NAMES = [
    # Pneumatics
    'arm_roller_bar_joint',
    'top_slider_joint',
    'top_gripper_left_arm_joint',
    'bottom_gripper_left_arm_joint',
    # Wheels
    'elevator_center_joint',
    'bottom_intake_joint',
]

class ROSNode(Node):
    def __init__(self):
        super().__init__('quickpublisher')
        self.publish_positions = [0.0]*6
        self.publisher = self.create_publisher(JointState, "/real/real_arm_commands", 10)
        self.publishTimer = self.create_timer(0.5, self.publish)
    
    def publish(self):
        msg = JointState()
        msg.name = JOINT_NAMES
        msg.position = self.publish_positions
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ROSNode()
    Thread(target=rclpy.spin, args=(node,)).start()
    while True:
        try:
            joint_index = int(input("Joint index: "))
            position = float(input("Position: "))
            node.publish_positions[joint_index] = position
        except:
            print("Invalid input")
            continue

if __name__ == '__main__':
    main()