import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import rosbag2_py
from pathlib import Path
from rclpy.serialization import deserialize_message
import rosbag2_py
from std_msgs.msg import String



class StageSubscriber(Node):

    def __init__(self):
        super().__init__('stage_subscriber')

        # self.subscription  # prevent unused variable warning
        self.reader = rosbag2_py.SequentialReader()
        self.storage_options = rosbag2_py.StorageOptions(uri='/workspaces/edna2023/src/frc_auton/frc_auton/bags/swerve_bag_0', storage_id='sqlite3') #change this to the bag you want to read
        self.converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr')
        self.reader.open(self.storage_options,self.converter_options)
        self.reader_2 = rosbag2_py.SequentialReader()
        self.storage_options_2 = rosbag2_py.StorageOptions(uri='/workspaces/edna2023/src/frc_auton/frc_auton/bags/arm_bag_0', storage_id='sqlite3') #change this to the bag you want to read
        self.converter_options_2 = rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr')
        self.reader_2.open(self.storage_options_2,self.converter_options_2)
        self.subscription = self.create_subscription(String,'frc_stage',self.listener_callback,10)
        self.publish_twist = self.create_publisher(Twist,'/real/swerve_controller/cmd_vel_unstamped',10)
        self.publish_trajectory = self.create_publisher(JointTrajectory,'/real/joint_trajectory_controller/joint_trajectory',10)

           


    def listener_callback(self, msg):
        stage = str(msg.data).split(" | ")[0]
        fms = bool(str(msg.data).split(" | ")[1])
        if(stage.lower() == 'auton' and fms):
            
            if(self.reader.has_next()):
                (topic, data, t)=self.reader.read_next()
                msg2 = deserialize_message(data,Twist)
                self.publish_twist.publish(msg2)
            
            if(self.reader_2.has_next()):
                (topic, data, t)=self.reader_2.read_next()
                msg3 = deserialize_message(data,JointTrajectory)
                self.publish_trajectory.publish(msg3)



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
