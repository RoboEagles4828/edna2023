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
import os



class StageSubscriber(Node):

    def __init__(self):
        super().__init__('stage_subscriber')

        

        file_counter= int(len(os.listdir('/workspaces/edna2023/src/frc_auton/frc_auton/Auto_ros_bag')))-1
        # self.reader = rosbag2_py.SequentialReader()
        # self.converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr')
        # self.reader.open(self.storage_options,self.converter_options)
        if file_counter != -1:
            
            self.storage_options = rosbag2_py.StorageOptions(uri='/workspaces/edna2023/src/frc_auton/frc_auton/Auto_ros_bag/bag_'+str(file_counter), storage_id='sqlite3') #change this to the bag you want to read
            self.playerOptions = rosbag2_py.PlayOptions()
            self.player = rosbag2_py.Player()

            self.subscription = self.create_subscription(String,'frc_stage',self.listener_callback,10)
            self.publish_twist = self.create_publisher(Twist,'swerve_controller/cmd_vel_unstamped',10)
            self.publish_trajectory = self.create_publisher(JointTrajectory,'joint_trajectory_controller/joint_trajectory',10)

            
            self.changed_stage = False
            self.stage= "Teleop"
            self.fms = "False"
            self.disabled = "True"
            self.has_bag_played = False


    def listener_callback(self, msg):
    
        stage = str(msg.data).split("|")[0]
        if stage != self.stage:
            self.changed_stage = True
            self.stage = stage
            self.has_bag_played = False
        fms = str(msg.data).split("|")[1]
        disabled = str(msg.data).split("|")[2]
        # self.get_logger().info('Subscription_stage: %b' % stage.lower() == 'auton' and fms)

        if(stage.lower() == 'auton' and disabled == "False" and self.changed_stage and not self.has_bag_played ):# and fms == 'True' ):
            # self.get_logger().info('Subscription_stage: %s' % 'auton')
            self.has_bag_played = True
            self.playerOptions
            self.player.play(self.storage_options,self.playerOptions)

    
    





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
