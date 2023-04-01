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
from time import time
import yaml
import math
from edna_interfaces.srv import SetBool



class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.client = self.create_client(SetBool, 'reset_field_oriented')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = SetBool.Request()

    def send_request(self, request):
        self.request.data = request
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class StageSubscriber(Node):

    def __init__(self):
        super().__init__('stage_subscriber')
        
        self.curr_file_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.curr_file_path, "../../../.."))
        self.package_root = os.path.join(self.project_root_path, 'src/frc_auton')
        # minimal_client = MinimalClientAsync()

        # response = minimal_client.send_request(True)
        # minimal_client.get_logger().info(
        # 'Result of add_two_ints: for %d + %d = %s' %
        # (True, response.success, response.message))
        # rclpy.spin_once(minimal_client)
        # minimal_client.destroy_node()
        file_counter= int(len(os.listdir(f'{self.package_root}/frc_auton/Auto_ros_bag')))-1
        # self.reader = rosbag2_py.SequentialReader()
        # self.converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr')
        # self.reader.open(self.storage_options,self.converter_options)
        if file_counter != -1:
            
            self.subscription = self.create_subscription(String,'frc_stage',self.listener_callback,10)
            self.publish_twist = self.create_publisher(Twist,'swerve_controller/cmd_vel_unstamped',10)
            self.publish_trajectory = self.create_publisher(JointTrajectory,'joint_trajectory_controller/joint_trajectory',10)
            
            self.changed_stage = False
            self.stage= ""
            self.fms = "False"
            self.isdisabled = "True"
            self.doAuton = False
            
            self.cmd = Twist()
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0
            self.timeInSeconds = 2.0

            self.taxiTimeDuration = 2.0


            # joint trajectory msg stuff
            self.joints = [
            'arm_roller_bar_joint',
            'elevator_center_joint',
            'elevator_outer_1_joint',
            'elevator_outer_2_joint',
            'top_gripper_right_arm_joint',
            'top_gripper_left_arm_joint',
            'top_slider_joint',
            'bottom_intake_joint',
            ]

            self.cmds: JointTrajectory = JointTrajectory()
            self.cmds.joint_names = self.joints
            self.position_cmds = JointTrajectoryPoint()
            self.cmds.points = [self.position_cmds]
            self.cmds.points[0].positions = [0.0] * len(self.joints)

            # yaml
            self.curr_file_path = os.path.abspath(__file__)
            self.project_root_path = os.path.abspath(os.path.join(self.curr_file_path, "../../../.."))
            self.yaml_path = os.path.join(self.project_root_path, 'src/edna_bringup/config/teleop-control.yaml')
            with open(self.yaml_path, 'r') as f:
                self.yaml = yaml.safe_load(f)
            self.joint_map = self.yaml['joint_mapping']
            self.joint_limits = self.yaml["joint_limits"]

            # task times
            self.tasks = [
                { 'dur': 0.25, 'task': self.gripperManager, 'arg': 0 },
                { 'dur': 0.5, 'task': self.armHeightManager, 'arg': 1 },
                { 'dur': 3.5, 'task': self.armExtensionManager, 'arg': 1 },
                { 'dur': 0.5, 'task': self.gripperManager, 'arg': 1 },
                { 'dur': 2.5, 'task': self.armExtensionManager, 'arg': 0 },
                { 'dur': 0.5, 'task': self.armHeightManager, 'arg': 0 },
                { 'dur': 3.0, 'task': self.goBackwards, 'arg': -1.5 },
                { 'dur': 0.1, 'task': self.stop, 'arg': 0 },
                { 'dur': 2.1, 'task': self.turnAround, 'arg': math.pi / 2 },
                { 'dur': 0.1, 'task': self.stop, 'arg': 0 },
            ]

            self.conePlacementDuration = 0
            for task in self.tasks:
                self.conePlacementDuration += task['dur']


            # TURN AROUND STUFF
            self.turnCmd = Twist()
            self.turnCmd.linear.x = 0.0
            self.turnCmd.linear.y = 0.0
            self.turnCmd.angular.z = 0.0
            self.turnTimeDuration = 2.0



    def flip_camera(self):
        minimal_client = MinimalClientAsync()
        response = minimal_client.send_request(True)
        minimal_client.destroy_node()

    def initAuton(self):
        self.startTime = time()
        self.turnStartTime = self.startTime + self.conePlacementDuration + 2
        self.changed_stage = False
        self.doAuton = True
        self.flip_camera()

        self.get_logger().info(f"STARTED AUTON AT {self.startTime}")

    
    def loopAuton(self):
        # self.taxiAuton()
        self.coneAuton()
        #self.turnAuton()


    # CONE AUTOMATION STUFF
    def publishCurrent(self):
        self.cmds.points = [self.position_cmds]
        self.publish_trajectory.publish(self.cmds)

    def armExtensionManager(self, pos):
        value = ''
        if(pos == 0):
            # RETRACT THE ARM
            value = 'min'
            self.get_logger().warn("ARM RETRACTED")
        elif(pos == 1):
            # EXTEND THE ARM
            value = 'max'
            self.get_logger().warn("ARM EXTENDED")

        self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = self.joint_limits["elevator_center_joint"][value]
        self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = self.joint_limits["elevator_outer_2_joint"][value]
        self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = self.joint_limits["top_slider_joint"][value]
        self.publishCurrent()
        

    def armHeightManager(self, pos):
        value = ''
        if(pos == 0):
            # LOWER THE ARM
            value = "min"
            self.get_logger().warn("ARM LOWERED")
        elif(pos == 1):
            # RAISE THE ARM
            value = "max"
            self.get_logger().warn("ARM RAISED")
        
        self.position_cmds.positions[int(self.joint_map['arm_roller_bar_joint'])] = self.joint_limits["arm_roller_bar_joint"][value]
        self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = self.joint_limits["elevator_outer_1_joint"][value]
        self.publishCurrent()

    
    def gripperManager(self, pos):
        value = ''
        if(pos == 1):
            # OPEN THE GRIPPER
            value = 'min'
            self.get_logger().warn("GRIPPER OPENED")
        elif(pos == 0):
            # CLOSE THE GRIPPER
            value = 'max'
            self.get_logger().warn("GRIPPER CLOSED")

        self.position_cmds.positions[int(self.joint_map['top_gripper_left_arm_joint'])] = self.joint_limits["top_gripper_right_arm_joint"][value]
        self.position_cmds.positions[int(self.joint_map['top_gripper_right_arm_joint'])] = self.joint_limits["top_gripper_right_arm_joint"][value]
        self.publishCurrent()

    def coneAuton(self):
        elapsedTime = time() - self.startTime

        totalDur = 0.0
        for task in self.tasks:
            totalDur += task['dur']
            if elapsedTime < totalDur:
                task['task'](task['arg'])
                return
    
    def taxiAuton(self):
        elapsedTime = time() - self.startTime
        if elapsedTime < self.taxiTimeDuration:
            self.cmd.linear.x = 0.5
            self.publish_twist.publish(self.cmd)
        else:
            return
        
    def stop(self, x):
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0
        self.publish_twist.publish(self.cmd)
        
    def goBackwards(self, speed):
        self.cmd.linear.x = speed
        self.publish_twist.publish(self.cmd)
        self.get_logger().warn("GOING BACKWARDS")

    def turnAround(self, angVel):
        self.turnCmd.angular.z = angVel
        self.publish_twist.publish(self.turnCmd)
        self.get_logger().warn("TURNING")

        
    
    
    def stopAuton(self):
        self.cmd.linear.x = 0.0
        # Publish twice to just to be safe
        self.publish_twist.publish(self.cmd)
        self.publish_twist.publish(self.cmd)
        self.get_logger().info(f"STOPPED AUTON AT {time()}"),
        self.doAuton = False


    def listener_callback(self, msg):
        
        # Check when any state has changed, enabled, disabled, auton, teleop, etc.
        stage = str(msg.data).split("|")[0]
        isdisabled = str(msg.data).split("|")[2]
        if stage != self.stage or isdisabled != self.isdisabled:
            self.changed_stage = True
            self.stage = stage
            self.isdisabled = isdisabled
        # fms = str(msg.data).split("|")[1]

        # Execute auton actions
        if(stage.lower() == 'auton' and self.isdisabled == "False"):# and fms == 'True' ):
            if self.changed_stage:
                self.initAuton()
            
            if self.doAuton:
                self.loopAuton()
        # We have moved out of auton enabled mode so stop if we are still running
        else:
            if self.doAuton:
                self.stopAuton()

                



def main(args=None):
    rclpy.init(args=args)

    # minimal_client = MinimalClientAsync()
    # response = minimal_client.send_request(True)
    # # minimal_client.get_logger().info(
    # #     'Result of add_two_ints: for %d + %d = %s' %
    # #     (True, response.success, response.message))

    # # minimal_client.destroy_node()
    # minimal_client.destroy_node()

   
    stage_subscriber = StageSubscriber()
    # stage_subscriber.send_request()

    rclpy.spin(stage_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stage_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# create reader instance and open for reading
