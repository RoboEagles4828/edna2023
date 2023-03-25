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
from writer_srv.srv import StartWriter
from threading import Thread

import rosbag2_py
# create writer instance and open for writing
class StartWriting(Node):
    def __init__(self):
        super().__init__('start_writer')
        self.subscription_stage =self.create_subscription(String, 'frc_stage', self.stage_callback, 10)
        self.srv = self.create_service(StartWriter, 'start_writer', self.service_callback)
        
        self.stage = ""
        self.fms = "False"
        self.is_disabled = "True"
        self.service_enabled = False

        self.curr_file_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.curr_file_path, "../../../.."))
        self.package_root = os.path.join(self.project_root_path, 'src/frc_auton')

        file_counter= int(len(os.listdir(f'{self.package_root}/frc_auton/Auto_ros_bag')))
        self.path = f'{self.package_root}/frc_auton/Auto_ros_bag/bag_'+str(file_counter)
       
        self.writer = rosbag2_py.SequentialWriter()
        self.storage_options = rosbag2_py._storage.StorageOptions(uri=self.path,storage_id='sqlite3')
        self.converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.swerve_record_options = rosbag2_py._transport.RecordOptions()
        self.arm_record_options = rosbag2_py._transport.RecordOptions()

        self.recorder = rosbag2_py.Recorder()
        self.swerve_record_options.topics = ['swerve_controller/cmd_vel_unstamped', 'joint_trajectory_controller/joint_trajectory']

        self.is_recording = False

    def service_callback(self, request, response):
        self.service_enabled = request.record
        self.kill = request.kill
        self.get_logger().info(f'Kill: {self.kill}')

        if(self.service_enabled):
            self.start_bag_writer()
        if(self.kill):
            self.get_logger().info(f'Kill: {self.kill}')

            self.stop_record_thread()
            self.get_logger().info("STOPPING RECORD THREAD")
        self.get_logger().info(f'Service Enabled: {self.service_enabled}')
        response.recording = True
        response.path = self.path
        return response
    def start_bag_writer(self):
        if (self.stage.lower() == "teleop" or self.stage.lower() == "auton") and (self.fms=='True' or self.service_enabled) and self.is_disabled=='False' and self.kill == False:
            self.start_record_thread()
            self.get_logger().info("STARTING RECORD THREAD")
        elif self.kill:
            self.get_logger().info(f'Kill: {self.kill}')
            self.stop_record_thread()
            self.get_logger().info("STOPPING RECORD THREAD")
    def stage_callback(self, msg):
        data = str(msg.data).split('|')
        self.stage = str(data[0])
        self.fms = str(data[1])
        self.is_disabled = str(data[2])
        # self.get_logger().info(f"THREAD STATUS: {self.bag_writer.get_thread_status()}")

    def bag_record(self):
        if self.is_recording == False:
            self.is_recording = True
            self.recorder.record(self.storage_options, self.swerve_record_options)

    def start_record_thread(self):
        self.record_thread = Thread(target=self.bag_record, daemon=True)
        self.is_recording = True
        self.record_thread.start()

    def stop_record_thread(self):
        self.is_recording = False
        self.record_thread.join()
    
    def get_thread_status(self):
        return self.record_thread.is_alive()


    
def main(args=None):    
    rclpy.init(args=args)

    service_writer = StartWriting()

    rclpy.spin(service_writer)
    

    service_writer.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()