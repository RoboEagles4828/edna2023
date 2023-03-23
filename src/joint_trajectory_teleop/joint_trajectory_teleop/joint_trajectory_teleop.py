import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy import logging
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import os
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

import time
import yaml

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

        # Joint Map
        self.joints = [
            'arm_roller_bar_joint',
            'elevator_center_joint',
            'elevator_outer_1_joint',
            'elevator_outer_2_joint',
            'top_gripper_right_arm_joint',
            'top_gripper_left_arm_joint',
            'top_slider_joint',
            'bottom_intake_joint',
        ]


        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.subscriber = self.create_subscription(Joy, 'joy', self.controller_callback, 10)
        self.timer_period = 0.5  # seconds

        # Load yaml
        self.yaml_path = '/workspaces/edna2023/src/edna_bringup/config/teleop-control.yaml'
        with open(self.yaml_path, 'r') as f:
            self.yaml = yaml.safe_load(f)

        # Macros
        self.functions = [self.elevator_loading_station, self.skis_up, self.elevator_mid_level, self.elevator_high_level, self.top_gripper_control, self.elevator_pivot_control, self.top_slider_control]

        # Variables
        self.cmds: JointTrajectory = JointTrajectory()
        self.position_cmds: JointTrajectoryPoint = JointTrajectoryPoint()
        self.position_cmds.positions = [0.0] * len(self.joints)
        self.cmds.joint_names = self.joints
        self.joint_map = self.yaml['joint_mapping']
        self.logger = logging.get_logger('JOINT-TRAJCECTORY-TELEOP')
        self.toggle_buttons = {}
        self.last_cmd = JointTrajectory()

        # Create Toggle Buttons
        for function in self.functions:
            button = self.yaml['controller_mapping'][self.yaml['function_mapping'][function.__name__]['button']]
            toggle = self.yaml['function_mapping'][function.__name__]['toggle']

            if toggle:
                self.toggle_buttons[function.__name__] = toggleButton(button)

    def controller_callback(self, joystick: Joy):
        for function in self.functions:
            button = self.yaml['controller_mapping'][self.yaml['function_mapping'][function.__name__]['button']]
            toggle = self.yaml['function_mapping'][function.__name__]['toggle']
            if toggle:
                button = self.toggle_buttons[function.__name__].toggle(joystick.buttons)
            else:
                button = joystick.buttons[button]
            function(button)
            if not self.is_equal(self.cmds, self.last_cmd):
                self.publisher_.publish(self.cmds)
                self.get_logger().info('Publishing...')
                self.last_cmd = self.cmds






    # Macros
    def elevator_loading_station(self, button_val: int):
        
        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = 0.1
            self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = 0.1
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = 1.0
        elif button_val == 0.0:
            self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = 0.0
            self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = 0.0
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = 0.0
        
        self.cmds.points = [self.position_cmds]

    def skis_up(self, button_val: int):

        #TODO: Tweak the values
        self.position_cmds.positions[int(self.joint_map['bottom_intake_joint'])] = button_val
        
        
        self.cmds.points = [self.position_cmds]

    def elevator_mid_level(self, button_val: int):
        
        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = 0.60
            self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = 0.60
            self.publisher_.publish(self.cmds)
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = 1.0
        
        
        self.cmds.points = [self.position_cmds]

    def elevator_high_level(self, button_val: int):
        
        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = 1.04
            self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = 1.04
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = 1.0
        elif button_val == 0.0:
            self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = 0.0
        
        self.cmds.points = [self.position_cmds]

    def top_gripper_control(self, button_val: int):

        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['top_gripper_left_arm_joint'])] = 1.0
            self.position_cmds.positions[int(self.joint_map['top_gripper_right_arm_joint'])] = 1.0
        elif button_val == 0.0:
            self.position_cmds.positions[int(self.joint_map['top_gripper_left_arm_joint'])] = 0.0
            self.position_cmds.positions[int(self.joint_map['top_gripper_right_arm_joint'])] = 0.0
        
        self.cmds.points = [self.position_cmds]

    def elevator_pivot_control(self, button_val: int):

        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['arm_roller_bar_joint'])] = 1.0
            self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = 0.2
        else:
            self.position_cmds.positions[int(self.joint_map['arm_roller_bar_joint'])] = 0.0
            self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = 0.0
        
        self.cmds.points = [self.position_cmds]

    def top_slider_control(self, button_val: int):

        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = 1.0
        
        self.cmds.points = [self.position_cmds]

    def is_equal(self, a: JointTrajectory, b: JointTrajectory) -> bool:
        if not isinstance(a.__class__, b.__class__):
            return False
        if a.joint_names != b.joint_names:
            return False
        if len(a.points) != len(b.points):
            return False
        for i in range(len(a.points)):
            if list(a.points[i].positions) != list(b.points[i].positions):
                return False
        return True






def main(args=None):
    rclpy.init(args=args)

    node = PublishTrajectoryMsg()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()