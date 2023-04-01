from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from edna_interfaces.srv import SetBool

import rosbag2_py
# create writer instance and open for writing
class StartWriting(Node):
    def __init__(self):
        super().__init__('start_writer')
        self.subscription_stage =self.create_subscription(String, 'frc_stage', self.stage_callback, 10)
        self.srv = self.create_service(SetBool, 'set_bool', self.service_callback)
        
        self.stage = ""
        self.fms = "False"
        self.is_disabled = "True"
        self.service_enabled = False

    def service_callback(self, request, response):
        self.bag_writer = BagWriter()
        self.service_enabled = request.data
        if(self.service_enabled):
            self.start_bag_writer()
        self.get_logger().info(f'Service Enabled: {self.service_enabled}')
        response.sucess = True
        response.message = self.bag_writer.path
        return response
    def start_bag_writer(self):
        
        if (self.stage.lower() == "teleop" or self.stage.lower() == "auton") and (self.fms=='True' or self.service_enabled) and self.is_disabled=='False':
            rclpy.spin(self.bag_writer)
            self.bag_writer.destroy_node()
            rclpy.shutdown()
    def stage_callback(self, msg):
        data = str(msg.data).split('|')
        self.stage = (data[0])
        self.fms = str(data[1])
        self.is_disabled = str(data[2])
      

class BagWriter(Node): 
    def __init__(self):
        super().__init__('bag_writer')
        self.curr_file_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.curr_file_path, "../../../.."))
        self.package_root = os.path.join(self.project_root_path, 'src/frc_auton')

        self.subscription_arm = self.create_subscription(JointTrajectory,'joint_trajectory_controller/joint_trajectory',self.arm_callback,10)
        self.subscription_swerve = self.create_subscription(Twist,'swerve_controller/cmd_vel_unstamped',self.swerve_callback,10)

        file_counter= int(len(os.listdir(f'{self.package_root}/frc_auton/Auto_ros_bag')))
        self.path = f'{self.package_root}/frc_auton/Auto_ros_bag/bag_'+str(file_counter)
       
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(uri=self.path,storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        topic_info_arm = rosbag2_py._storage.TopicMetadata(name=self.subscription_arm.topic_name,type='trajectory_msgs/msg/JointTrajectory',serialization_format='cdr')
        self.writer.create_topic(topic_info_arm)

        topic_info_swerve = rosbag2_py._storage.TopicMetadata(name=self.subscription_swerve.topic_name,type='geometry_msgs/msg/Twist',serialization_format='cdr')
        self.writer.create_topic(topic_info_swerve)
        
    def swerve_callback(self, msg):
            self.writer.write(
                self.subscription_swerve.topic_name,
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
                
    def arm_callback(self, msg):
            self.writer.write(
                self.subscription_arm.topic_name,
                serialize_message(msg),
                self.get_clock().now().nanoseconds)


    
def main(args=None):    
    rclpy.init(args=args)

    service_writer = StartWriting()

    rclpy.spin(service_writer)

    service_writer.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()