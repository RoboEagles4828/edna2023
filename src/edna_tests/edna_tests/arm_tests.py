import rclpy
import time

from .joint_test import TesterNode




# ROS Topics
SUBSCRIBE_TOPIC_NAME = '/real/real_joint_states'
PUBLISH_TOPIC_NAME = '/real/real_arm_commands'
PUBLISH_INTERVAL = .5 # seconds

# Tolerances
PASS_TOLERANCE = 0.5
WARN_TOLERANCE = 1

TESTS = [
    # Here's where you define your tests, in the same style as this one.
    # {"positions": [0.0]*6, "time": 3.0},

    # Toggle pistons one at a time (to true)
    {"positions": [0.0]*4 + [0.0] + [0.0], "time": 10.0},
    # {"positions": [0.0]*4 + [0.5] + [0.0], "time": 10.0},
    # {"positions": [0.0]*4 + [1.0] + [0.0], "time": 10.0},
    # {"positions": [0.0]*6, "time": 10.0},

]

JOINT_NAMES = [
    # Pneumatics
    'arm_roller_bar_joint',
    'top_slider_joint',
    'top_gripper_left_arm_joint',
    # Wheels
    'elevator_center_joint',
    'bottom_intake_joint',
]

def main():
    rclpy.init()
    testerNode = TesterNode(
        tests=TESTS, 
        joint_names=JOINT_NAMES, 
        joint_range=[8, 14],
        pub_topic_name=PUBLISH_TOPIC_NAME, 
        sub_topic_name=SUBSCRIBE_TOPIC_NAME, 
        pub_interval=PUBLISH_INTERVAL, 
        pass_tolerance=PASS_TOLERANCE, 
        warn_tolerance=WARN_TOLERANCE)
    rclpy.spin(testerNode)

if __name__ == '__main__':
    main()