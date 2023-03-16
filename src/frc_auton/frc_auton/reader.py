from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rosbags.typesys.types import geometry_msgs__msg__Twist, trajectory_msgs__msg__JointTrajectory
from std_msgs.msg import String


class StageSubscriber(Node):

    def __init__(self):
        super().__init__('stage_subscriber')
        self.subscription = self.create_subscription(String,'frc_stage',self.listener_callback,10)
        self.publish_twist = self.create_publisher(Twist,'/real/swerve_controller/cmd_vel_unstamped',10)
        self.publish_trajectory = self.create_publisher(JointTrajectory,'/real/joint_trajectory_controller/joint_trajectory',10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # print(msg)
        # if msg==True:
        # print("yes")
        if(msg.data=="Auton"):
            with Reader('Auto_ros_bag/rosbag2_2023_03_14-23_54_20') as reader:
                for connection, timestamp, rawdata in reader.messages():
                    # print(connection.topic)
                    if connection.topic == '/real/swerve_controller/cmd_vel_unstamped':
                        msg2 = deserialize_cdr(rawdata,connection.msgtype)
                        twist_msg = Twist()
                        twist_msg.linear.x = float(msg2.linear.x)
                        twist_msg.linear.y = float(msg2.linear.y)
                        twist_msg.angular.z = float(msg2.angular.z) 
                        self.publisher.publish(twist_msg)
                    if connection.topic == '/real/joint_trajectory_controller/joint_trajectory':
                        msg = deserialize_cdr(rawdata,connection.msgtype)
                        trajectory_msg= JointTrajectory()
                        trajectory_msg.joint_names = str(msg2.joint_names)
                        trajectory_msg.points.positions = str(msg2.points.positions)
                        



def main(args=None):
    rclpy.init(args=args)

    stage_subscriber = StageSubscriber()

    rclpy.spin(stage_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stage_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# create reader instance and open for reading
