import rclpy
from rclpy.node import Node
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import os

class PublishTrajectoryMsg(Node):

    def __init__(self):
        super().__init__('publish_trajectory_msg')

        # self.declare_parameters(
        #     parameters = [{
        #         "arm_roller_bar_joint": "arm_roller_bar_joint",
        #         "elevator_left_elevator_center_joint": "elevator_left_elevator_center_joint"
        #     }]
        # )

        self.joints = [
            'arm_roller_bar_joint', 
            'elevator_left_elevator_center_joint',
            'elevator_left_elevator_outer_1_joint',
            'elevator_left_elevator_outer_2_joint',
            'elevator_right_elevator_center_joint',
            'elevator_right_elevator_outer_1_joint',
            'elevator_right_elevator_outer_2_joint'
        ]

        self.NAMESPACE = f"{os.environ.get('ROS_NAMESPACE')}" if 'ROS_NAMESPACE' in os.environ else 'default'

        self.publisher_ = self.create_publisher(JointTrajectory, f'/{self.NAMESPACE}/joint_trajectory_controller/joint_trajectory', 10)
        self.subscriber = self.create_subscription(Joy, f'/{self.NAMESPACE}/joy', self.controller_callback, 10)
        timer_period = 0.5  # seconds
        self.i = 0

    def controller_callback(self, msg: Joy):
        cmds = JointTrajectory()
        position_cmds = JointTrajectoryPoint()
        position_cmds.positions = [float(not msg.buttons[5]), float(not msg.buttons[2]), float(not msg.buttons[5]), float(not msg.buttons[2]), float(not msg.buttons[2]), float(not msg.buttons[5]), float(not msg.buttons[2])]

        
        cmds.joint_names = self.joints
        cmds.points = [position_cmds]
        
        self.publisher_.publish(cmds)
        self.get_logger().info('Publishing...')
        self.i += 1


# 'front_left_wheel_joint',
#             'front_left_axle_joint',
#             'front_right_wheel_joint',
#             'front_right_axle_joint',
#             'rear_left_wheel_joint',
#             'rear_left_axle_joint',
#             'rear_right_wheel_joint',
#             'rear_right_axle_joint']

def main(args=None):
    rclpy.init(args=args)

    node = PublishTrajectoryMsg()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()