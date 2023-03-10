import rclpy
from rclpy.node import Node
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import os
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

import time

class PublishTrajectoryMsg(Node):

    def __init__(self):
        super().__init__('publish_trajectory_msg')

        self.joints = [
            'arm_roller_bar_joint',
            'elevator_center_joint',
            'elevator_outer_1_joint',
            'elevator_outer_2_joint',
            'top_gripper_right_arm_joint',
            'top_gripper_left_arm_joint',
            'top_slider_joint',
            'bottom_intake_joint',
            'bottom_gripper_right_arm_joint',
            'bottom_gripper_left_arm_joint',
        ]

        self.button_dict = {
            'A': 0,
            'B': 1,
            'X': 2,
            'Y': 3,
            'LB': 4,
            'RB': 5,
            'MENU': 7,
            'SQUARES': 6,
            'RIN': 10
        }
        self.axis_dict = {
            'DPAD_Y': 7,
            'DPAD_X': 6,
            'LT': 2,
            'RT': 5,
        }

        self.NAMESPACE = f"{os.environ.get('ROS_NAMESPACE')}" if 'ROS_NAMESPACE' in os.environ else 'default'

        self.pos = 0.0
        self.rot = 0.0
        # qos = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)#qos_profile=qos)
        self.subscriber = self.create_subscription(Joy, 'joy', self.controller_callback, 10)#qos_profile=qos)
        self.timer_period = 0.5  # seconds

    def controller_callback(self, joystick: Joy):
        cmds = JointTrajectory()
        position_cmds = JointTrajectoryPoint()
        # self.get_logger().info('\nBUTTONS: ' + str(joystick.buttons) + '\nAXES: ' + str(joystick.axes))

        x_flag = False
        y_flag = False
        y_flag_negative = False
        x_flag_negative = False

        # c
        if y_flag:
            self.pos = 1.0
        elif y_flag_negative:
            self.pos = 0.0
        elif x_flag:
            self.pos = 0.2
        elif x_flag_negative:
            self.pos = 0.4

        if joystick.buttons[self.button_dict['RB']] == 1.0:
            self.rot = 0.1
        else:
            self.rot = 0.0

        

        position_cmds.positions = [
            float(joystick.buttons[self.button_dict['RB']]),
            self.pos,
            self.rot,
            self.pos,
            float(joystick.buttons[self.button_dict['X']]),
            float(joystick.buttons[self.button_dict['X']]),
            float(joystick.buttons[self.button_dict['B']]),
            float(joystick.buttons[self.button_dict['A']]),
            float(joystick.buttons[self.button_dict['Y']]),
            float(joystick.buttons[self.button_dict['Y']]),
        ]
        
        cmds.joint_names = self.joints
        cmds.points = [position_cmds]
        
        self.publisher_.publish(cmds)
        self.get_logger().info('Publishing...')
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)

    node = PublishTrajectoryMsg()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()