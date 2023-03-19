from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr, deserialize_cdr
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rosbags.typesys.types import geometry_msgs__msg__Twist, trajectory_msgs__msg__JointTrajectory, trajectory_msgs__msg__JointTrajectoryPoint
from geometry_msgs.msg import Twist
import time
import rclpy
from rclpy.node import Node
import os
import numpy as np
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String

import rosbag2_py
# create writer instance and open for writing
class BagWriter(Node): 
    def __init__(self):
        super().__init__('bag_writer')
        self.stage = ""
        self.fms = False
        self.subscription_stage =self.create_subscription(String, '/frc_stage', self.stage_callback, 10)
        file_counter= int(len(os.listdir('/workspaces/edna2023/src/frc_auton/frc_auton/bags'))/2)
        self.swerve_path = '/workspaces/edna2023/src/frc_auton/frc_auton/bags/swerve_bag_'+str(file_counter)
        self.arm_path = '/workspaces/edna2023/src/frc_auton/frc_auton/bags/arm_bag_'+str(file_counter)
        #creates writer for arm bag
        self.writer_arm = rosbag2_py.SequentialWriter()
        storage_options_arm = rosbag2_py._storage.StorageOptions(uri=self.arm_path,storage_id='sqlite3')
        converter_options_arm = rosbag2_py._storage.ConverterOptions('', '')
        self.writer_arm.open(storage_options_arm, converter_options_arm)
        topic_info_arm = rosbag2_py._storage.TopicMetadata(name='/real/joint_trajectory_controller/joint_trajectory',type='trajectory_msgs/msg/JOintTrajectory',serialization_format='cdr')
        self.writer_arm.create_topic(topic_info_arm)
        self.subscription_arm = self.create_subscription(JointTrajectory,'/real/joint_trajectory_controller/joint_trajectory',self.arm_callback,10)
        self.subscription_arm
        #creates writer for swerve bag
        self.writer_swerve=rosbag2_py.SequentialWriter()
        storage_options_swerve = rosbag2_py._storage.StorageOptions(uri=self.swerve_path,storage_id='sqlite3')
        converter_options_swerve = rosbag2_py._storage.ConverterOptions('', '')
        self.writer_swerve.open(storage_options_swerve, converter_options_swerve)
        topic_info_swerve = rosbag2_py._storage.TopicMetadata(name='/real/swerve_controller/cmd_vel_unstamped',type='geometry_msgs/msg/Twist',serialization_format='cdr')
        self.writer_swerve.create_topic(topic_info_swerve)
        self.subscription_swerve = self.create_subscription(Twist,'/real/swerve_controller/cmd_vel_unstamped',self.swerve_callback,10)
        self.subscription_swerve
    def swerve_callback(self, msg):
        if(self.stage == "auton" and self.fms):
            self.writer_swerve.write(
                '/real/swerve_controller/cmd_vel_unstamped',
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
                
    def arm_callback(self, msg):
        if(self.stage == "auton" and self.fms):
            self.writer_arm.write(
                '/real/joint_trajectory_controller/joint_trajectory',
                serialize_message(msg),
                self.get_clock().now().nanoseconds)


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