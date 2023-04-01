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


ENABLE_THROTTLE = True

class toggleButton():
    def __init__(self, button, isAxis=False):
        self.last_button = 0.0
        self.flag = False
        self.button = button
        self.isAxis = isAxis
    
    def toggle(self, buttons_list):
        currentButton = buttons_list[self.button]
        if self.isAxis:
            # currentButton = currentButton / 10000 if currentButton > 1 else currentButton
            if currentButton == -10000.0  and self.last_button != -10000.0:
                self.flag = not self.flag
                self.last_button = currentButton
                return self.flag
            else:
                self.last_button = currentButton
                return self.flag

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
        self.curr_file_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.curr_file_path, "../../../.."))
        self.yaml_path = os.path.join(self.project_root_path, 'src/edna_bringup/config/teleop-control.yaml')
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
        self.joint_limits = self.yaml["joint_limits"]

        # Create Toggle Buttons
        for function in self.functions:
            buttonName = self.yaml['function_mapping'][function.__name__]['button']
            button = self.yaml['controller_mapping'][buttonName]
            toggle = self.yaml['function_mapping'][function.__name__]['toggle']
            isAxis = "axis" in buttonName.lower()

            if toggle:
                self.toggle_buttons[function.__name__] = toggleButton(button, isAxis)

    def controller_callback(self, joystick: Joy):
        for function in self.functions:
            buttonName = self.yaml['function_mapping'][function.__name__]['button']
            button = self.yaml['controller_mapping'][buttonName]
            toggle = self.yaml['function_mapping'][function.__name__]['toggle']

            if toggle:
                tglBtn = self.toggle_buttons[function.__name__]
                if tglBtn.isAxis:
                    button = tglBtn.toggle(joystick.axes)
                else:
                    button = tglBtn.toggle(joystick.buttons)
            else:
                button = joystick.buttons[button]
            function(button)
        self.publisher_.publish(self.cmds)






    # Macros
    def elevator_loading_station(self, button_val: int):
        
        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = 0.056
            self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = 0.056
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = self.joint_limits["top_slider_joint"]["max"]
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
            self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = 0.336
            self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = 0.336
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = self.joint_limits["top_slider_joint"]["max"]
        
        
        self.cmds.points = [self.position_cmds]

    def elevator_high_level(self, button_val: int):
        
        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = self.joint_limits["elevator_center_joint"]["max"]
            self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = self.joint_limits["elevator_outer_2_joint"]["max"]
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = self.joint_limits["top_slider_joint"]["max"]
        elif button_val == 0.0:
            self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = 0.0
        
        self.cmds.points = [self.position_cmds]

    def top_gripper_control(self, button_val: int):

        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['top_gripper_left_arm_joint'])] = self.joint_limits["top_gripper_left_arm_joint"]["max"]
            self.position_cmds.positions[int(self.joint_map['top_gripper_right_arm_joint'])] = self.joint_limits["top_gripper_right_arm_joint"]["max"]
        elif button_val == 0.0:
            self.position_cmds.positions[int(self.joint_map['top_gripper_left_arm_joint'])] = self.joint_limits["top_gripper_right_arm_joint"]["min"]
            self.position_cmds.positions[int(self.joint_map['top_gripper_right_arm_joint'])] = self.joint_limits["top_gripper_right_arm_joint"]["min"]
        
        self.cmds.points = [self.position_cmds]

    def elevator_pivot_control(self, button_val: int):

        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['arm_roller_bar_joint'])] = self.joint_limits["arm_roller_bar_joint"]["max"]
            self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = self.joint_limits["elevator_outer_1_joint"]["max"]
        else:
            self.position_cmds.positions[int(self.joint_map['arm_roller_bar_joint'])] = 0.0
            self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = 0.0
        
        self.cmds.points = [self.position_cmds]

    def top_slider_control(self, button_val: int):

        #TODO: Tweak the values
        if button_val == 1.0:
            self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = self.joint_limits["top_slider_joint"]["max"]
        
        self.cmds.points = [self.position_cmds]

def main(args=None):
    rclpy.init(args=args)

    node = PublishTrajectoryMsg()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()