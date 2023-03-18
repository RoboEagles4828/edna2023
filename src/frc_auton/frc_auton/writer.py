import rclpy 
from rclpy.node import Node
from rclpy.serialization import serialize_message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rosbags.rosbag2 import Writer
import rosbag2_py
import time

class BagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='teleop_bag',
            storage_id='sqlite3')
        self.subscription = self.create_subscription(
                Twist(),
                '/real/swerve_controller/cmd_vel_unstamped',
                self.topic_callback,
                10
        )
        self.subscription
        self.subscription = self.create_subscription(
                JointTrajectory(),
                '/real/joint_trajectory_controller/joint_trajectory',
                self.topic_callback,
                10)
        self.subscription
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

    def topic_callback(self, msg):
        topic_info = rosbag2_py._storage.TopicMetadata(
            name='rosbag_commands_cmd_vel',
            type=str(),
            serialization_format='str',
            )
        self.writer.create_topic(topic_info)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='rosbag_commands_joint_trajectory',
            type=str(),
            serialization_format='str',
            )
        self.writer.create_topic(topic_info)
        self.writer.write(
            '/real/swerve_controller/cmd_vel_unstamped',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        self.writer.write(
            '/real/joint_trajectory_controller/joint_trajectory',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

def main(args=None):
    rclpy.init(args=args)
    br = BagRecorder()
    rclpy.spin(br)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
