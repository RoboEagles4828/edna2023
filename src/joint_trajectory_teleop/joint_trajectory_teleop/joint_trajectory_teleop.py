import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import os

import time

class toggleButton():
    def __init__(self, button):
        self.last_button = 0.0
        self.flag = False
        self.button = button
        
    
    def toggle(self, buttons_list):
        currentButton = buttons_list[self.button]
        if currentButton == 1.0 and self.last_button == 0.0:
            self.flag = not self.flag
            self.last_button = currentButton
            return self.flag
        else:
            self.last_button = currentButton
            return self.flag
        

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

        self.pos = 0.0
        self.rot = 0.0

        # qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.subscriber = self.create_subscription(Joy, 'joy', self.controller_callback, 10)
        self.timer_period = 0.5  # seconds
        self.aButton = toggleButton(self.button_dict['A'])
        self.bButton = toggleButton(self.button_dict['B'])

    def controller_callback(self, joystick: Joy):
        cmds = JointTrajectory()
        position_cmds = JointTrajectoryPoint()
        # self.get_logger().info('\nBUTTONS: ' + str(joystick.buttons) + '\nAXES: ' + str(joystick.axes))

        if joystick.buttons[self.button_dict['LB']] == 1.0:
            self.pos = 1.1
        elif joystick.buttons[self.button_dict['RB']] == 1.0:
            self.pos = 1.5
        elif joystick.buttons[self.button_dict['RIN']] == 1.0:
            self.pos = 0.2
        elif joystick.buttons[self.button_dict['LB']] == 0.0:
            self.pos = 0.0
        elif joystick.buttons[self.button_dict['RB']] == 0.0:
            self.pos = 0.0
        elif joystick.buttons[self.button_dict['RIN']] == 1.0:
            self.pos

        if joystick.buttons[self.button_dict['Y']] == 1.0:
            self.rot = 0.1
        else:
            self.rot = 0.0
        

        
        aToggleValue = self.aButton.toggle(joystick.buttons)
        bToggleValue = self.bButton.toggle(joystick.buttons)

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
            float(joystick.buttons[self.button_dict['Y']]),
            self.pos,
            self.rot,
            self.pos,
            float(aToggleValue),
            float(aToggleValue),
            float(bToggleValue),
            float(joystick.buttons[self.button_dict['A']]),
            float(joystick.axes[self.axis_dict['RT']]),
            float(joystick.axes[self.axis_dict['RT']]),
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