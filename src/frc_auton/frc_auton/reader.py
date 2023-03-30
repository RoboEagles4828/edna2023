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



class StageSubscriber(Node):

    def __init__(self):
        super().__init__('stage_subscriber')

        
        self.curr_file_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.curr_file_path, "../../../.."))
        self.package_root = os.path.join(self.project_root_path, 'src/frc_auton')

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
                { 'dur': 1, 'task': self.raiseArm },
                { 'dur': 1, 'task': self.extendArm },
                { 'dur': 1, 'task': self.openGripper },
                { 'dur': 1, 'task': self.closeGripper },
                { 'dur': 1, 'task': self.retractArm },
                { 'dur': 1, 'task': self.lowerArm },
            ]

            self.prefixSumArr = [] * (len(self.tasks) + 1)

    
    def prefixSum(self):
        self.prefixSumArr[0] = 0
        for i in range(1, len(self.prefixSumArr)):
            self.prefixSumArr[i] = self.prefixSumArr[i-1] + self.tasks[i-1]['dur']



    def initAuton(self):
        self.startTime = time()
        self.changed_stage = False
        self.doAuton = True
        self.get_logger().info(f"STARTED AUTON AT {self.startTime}")
        self.get_logger().warn(str(self.prefixSumArr))

    
    def loopAuton(self):
        # self.taxiAuton()
        self.coneAuton()

    # CONE AUTOMATION STUFF
    def raiseArm(self):
        self.position_cmds.positions[int(self.joint_map['arm_roller_bar_joint'])] = self.joint_limits["arm_roller_bar_joint"]["max"]
        self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = self.joint_limits["elevator_outer_1_joint"]["max"]
        self.cmds.points = [self.position_cmds]
        self.publish_trajectory.publish(self.cmds)
        self.get_logger().warn("ARM RAISED")
    def extendArm(self):
        self.position_cmds.positions[int(self.joint_map['elevator_center_joint'])] = self.joint_limits["elevator_center_joint"]["max"]
        self.position_cmds.positions[int(self.joint_map['elevator_outer_2_joint'])] = self.joint_limits["elevator_outer_2_joint"]["max"]
        self.position_cmds.positions[int(self.joint_map['top_slider_joint'])] = self.joint_limits["top_slider_joint"]["max"]
        self.cmds.points = [self.position_cmds]
        self.publish_trajectory.publish(self.cmds)
        self.get_logger().warn("ARM EXTENDED")
    def openGripper(self):
        self.position_cmds.positions[int(self.joint_map['top_gripper_left_arm_joint'])] = self.joint_limits["top_gripper_left_arm_joint"]["min"]
        self.position_cmds.positions[int(self.joint_map['top_gripper_right_arm_joint'])] = self.joint_limits["top_gripper_right_arm_joint"]["min"]
        self.cmds.points = [self.position_cmds]
        self.publish_trajectory.publish(self.cmds)
        self.get_logger().warn("GRIPPER OPENED")
    def closeGripper(self):
        self.position_cmds.positions[int(self.joint_map['top_gripper_left_arm_joint'])] = self.joint_limits["top_gripper_right_arm_joint"]["max"]
        self.position_cmds.positions[int(self.joint_map['top_gripper_right_arm_joint'])] = self.joint_limits["top_gripper_right_arm_joint"]["max"]
        self.cmds.points = [self.position_cmds]
        self.publish_trajectory.publish(self.cmds)
        self.get_logger().warn("GRIPPER CLOSED")
    def retractArm(self):
        self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = 0.0
        self.cmds.points = [self.position_cmds]
        self.publish_trajectory.publish(self.cmds)
        self.get_logger().warn("ARM RETRACTED")
    def lowerArm(self):
        self.position_cmds.positions[int(self.joint_map['arm_roller_bar_joint'])] = 0.0
        self.position_cmds.positions[int(self.joint_map['elevator_outer_1_joint'])] = 0.0
        self.cmds.points = [self.position_cmds]
        self.publish_trajectory.publish(self.cmds)
        self.get_logger().warn("ARM LOWERED")


    def coneAuton(self):
        elapsedTime = time() - self.startTime

        totalDur = 0.0
        for task in self.tasks:
            totalDur += task['dur']
            if elapsedTime < totalDur:
                task['task']()
                return
    
    def taxiAuton(self):
        elapsedTime = time() - self.startTime
        if elapsedTime < self.taxiTimeDuration:
            self.cmd.linear.x = 0.5
            self.publish_twist.publish(self.cmd)
        else:
            return
        
    
    
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
