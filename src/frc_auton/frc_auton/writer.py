from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr
from rosbags.typesys.types import std_msgs__msg__String as String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rosbags.typesys.types import geometry_msgs__msg__Twist, trajectory_msgs__msg__JointTrajectory
from geometry_msgs.msg import Twist
import time
import rclpy
from rclpy.node import Node
# create writer instance and open for writing
class BagWriter(Node): 
    def __init__(self):
        super().__init__('bag_writer')
        self.subscription_swerve = self.create_subscription(Twist, 'swerve_controller/cmd_vel_unstamped', self.swerve_callback, 10)
        self.subscription_arm = self.create_subscription(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', self.arm_callback, 10)
    def swerve_callback(self, msg):
        #if msg.data.split('|')[1] == 'True':
        with Writer('teleop_bag') as writer:
            topic = 'swerve_controller/cmd_vel_unstamped'
            msgtype = geometry_msgs__msg__Twist.__msgtype__
            connection = writer.add_connection(topic, msgtype, 'cdr', '')

            # serialize and write message
            timestamp = time.time()
            message = msg
            writer.write(connection, timestamp, serialize_cdr(message, msgtype))
    def arm_callback(self, msg):
        with Writer('teleop_bag') as writer:
            topic = 'joint_trajectory_controller/joint_trajectory'
            msgtype = trajectory_msgs__msg__JointTrajectory.__msgtype__
            connection = writer.add_connection(topic, msgtype, 'cdr', '')

            # serialize and write message
            timestamp = time.time()
            message = msg
            writer.write(connection, timestamp, serialize_cdr(message, msgtype))
def main(args=None):    
    rclpy.init(args=args)

    bag_writer = BagWriter()

    rclpy.spin(bag_writer)

    bag_writer.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()