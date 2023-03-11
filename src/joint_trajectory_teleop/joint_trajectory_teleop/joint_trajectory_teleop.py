import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import os

import time

class PublishTrajectoryMsg(Node):

    def __init__(self):
        super().__init__('publish_trajectory_msg')

        # flags
        self.a = False
        self.b = False
        self.x = False
        self.y = False
        self.lb = False
        self.rb = False
        self.menu = False
        self.squares = False
        self.rin = False


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
        self.flipped_button_dict = {
            0: 'A',
            1: 'B',
            2: 'X',
            3: 'Y',
            4: 'LB',
            5: 'RB',
            7: 'MENU',
            6: 'SQAURES',
            10: 'RIN'

        }
        self.axis_dict = {
            'DPAD_Y': 7,
            'DPAD_X': 6,
            'LT': 2,
            'RT': 5,
        }

        self.flag_dict = {
            'A': self.a,
            'B': self.b,
            'X': self.x,
            'Y': self.y,
            'LB': self.lb,
            'RB': self.rb,
            'MENU': self.menu,
            'SQUARES': self.squares,
            'RIN': self.rin
        }

        self.pos = 0.0
        self.rot = 0.0

        # qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.subscriber = self.create_subscription(Joy, 'joy', self.controller_callback, 10)
        self.timer_period = 0.5  # seconds

    def controller_callback(self, joystick: Joy):
        cmds = JointTrajectory()
        position_cmds = JointTrajectoryPoint()
        # self.get_logger().info('\nBUTTONS: ' + str(joystick.buttons) + '\nAXES: ' + str(joystick.axes))

        for i in range(len(joystick.buttons)):
            if joystick.buttons[i] == 1.0:
                self.flag_dict[self.flipped_button_dict[i]] = not self.flag_dict[self.flipped_button_dict[i]]

        if self.flag_dict['LB']:     
            self.pos = 0.5
        elif self.flag_dict['RB']:
            self.pos = 1.0
        elif self.flag_dict['RIN']:
            self.pos = 0.2
        elif self.flag_dict['LB'] == False:
            self.pos = 0.0
        elif self.flag_dict['RB'] == False:
            self.pos = 0.0
        elif self.flag_dict['RIN'] == False:
            self.pos = 0.0

        if self.flag_dict['Y']:
            self.rot = 0.1
        else:
            self.rot = 0.0

        # self.joints = [
        #     'arm_roller_bar_joint',
        #     'elevator_center_joint',
        #     'elevator_outer_1_joint',
        #     'elevator_outer_2_joint',
        #     'top_gripper_right_arm_joint',
        #     'top_gripper_left_arm_joint',
        #     'top_slider_joint',
        #     'bottom_intake_joint',
        #     'bottom_gripper_right_arm_joint',
        #     'bottom_gripper_left_arm_joint',
        # ]

        position_cmds.positions = [
            1.0 if self.flag_dict['Y'] else 0.0,
            self.pos,
            self.rot,
            self.pos,
            1.0 if self.flag_dict['A'] else 0.0,
            1.0 if self.flag_dict['A'] else 0.0,
            float(joystick.buttons[self.button_dict['B']]),
            0.0,
            0.0,
            0.0
        ]
        
        cmds.joint_names = self.joints
        cmds.points = [position_cmds]
        
        self.publisher_.publish(cmds)
        # self.get_logger().info('Publishing...')

def main(args=None):
    rclpy.init(args=args)

    node = PublishTrajectoryMsg()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()