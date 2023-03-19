from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr, deserialize_cdr
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rosbags.typesys.types import geometry_msgs__msg__Twist, trajectory_msgs__msg__JointTrajectory
from geometry_msgs.msg import Twist
import time
import rclpy
from rclpy.node import Node
import os
import subprocess
# create writer instance and open for writing
class BagWriter(Node): 
    def __init__(self):
        super().__init__('bag_writer')
        self.stage = ""
        self.fms = False
        self.subscription_stage =self.create_subscription(String, '/frc_stage', self.stage_callback, 10)
        self.subscription_swerve = self.create_subscription(Twist, '/saranga/swerve_controller/cmd_vel_unstamped', self.swerve_callback, 10)
        self.subscription_arm = self.create_subscription(JointTrajectory, '/saranga/joint_trajectory_controller/joint_trajectory', self.arm_callback, 10)
        self.swerve_path = '/workspaces/edna2023/src/frc_auton/frc_auton/bags/swerve_bag'
        self.arm_path = '/workspaces/edna2023/src/frc_auton/frc_auton/bags/arm_bag'
    def swerve_callback(self, msg):
        self.get_logger().info('Subscription_swerve IS HAPPENING')
        if self.stage.lower() == 'teleop' and self.fms:
            if not os.path.exists(self.swerve_path):
                with Writer(self.swerve_path) as writer:
                    topic = 'swerve_controller/cmd_vel_unstamped'
                    twist_msg = geometry_msgs__msg__Twist(msg.linear, msg.angular)
                    msgtype = geometry_msgs__msg__Twist.__msgtype__
                    connection = writer.add_connection(topic, msgtype, 'cdr', '')

                    twist_msg.linear.x = float(msg.linear.x)
                    twist_msg.linear.y = float(msg.linear.y)
                    twist_msg.angular.z = float(msg.angular.z) 

                    # serialize and write message
                    timestamp = time.time()
                    writer.write(connection, timestamp, serialize_cdr(twist_msg, msgtype))
            else:
                self.swerve_path += "_1"
                
    def arm_callback(self, msg):
        self.get_logger().info('Subscription_arm IS HAPPENING')
        if self.stage.lower() == 'teleop' and self.fms:
            if not os.path.exists(self.arm_path):
                with Writer(self.arm_path) as writer:
                    topic = 'joint_trajectory_controller/joint_trajectory'
                    trajectory_msg = trajectory_msgs__msg__JointTrajectory(msg.header, msg.joint_names, msg.points)
                    msgtype = trajectory_msgs__msg__JointTrajectory.__msgtype__
                    connection = writer.add_connection(topic, msgtype, 'cdr', '')

                    trajectory_msg.joint_names = str(msg.joint_names)
                    trajectory_msg.points = str(msg.points)

                    # serialize and write message
                    timestamp = time.time()
                    writer.write(connection, timestamp, serialize_cdr(trajectory_msg, msgtype))
            else:
                self.arm_path += "_1"

    def stage_callback(self, msg):
        self.get_logger().info('Subscription_stage: %s' % msg.data)
        data = str(msg.data).split(' | ')
        self.stage = data[0]
        self.fms = bool(data[1])
        self.get_logger().info(f'Stage: {self.stage} FMS: {self.fms}')


def main(args=None):    
    rclpy.init(args=args)

    bag_writer = BagWriter()

    rclpy.spin(bag_writer)

    bag_writer.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()