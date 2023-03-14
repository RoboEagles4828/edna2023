import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import torch
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Runner(Node):
    def __init__(self):
        super().__init__("reinforcement_learning_runner")
        # self.robot_ip = robot_ip
        self.policy = torch.load("/workspaces/edna2023/isaac/Swervesim/swervesim/runs/SwerveCS/nn/SwerveCS.pth")
        self.joint_action_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.joint_trajectory_action_pub = self.create_publisher(Twist, "joint_trajectory_message", 10)
        self.odom_sub = self.create_subscription(Float32, "odom", self.odom_callback, 10)
        self.joint_state_sub = self.create_subscription(Float32, "joint_state", self.joint_state_callback, 10)
        self.odom_msg = Odometry()
        self.joint_state_msg = JointState()
        self.twist_msg = Twist()
        self.cmds = JointTrajectory()
        self.position_cmds = JointTrajectoryPoint()
        self.episode_reward = 0
        self.step = 0
        self.joints = [
            'arm_roller_bar_joint',
            'elevator_center_joint',
            'elevator_outer_1_joint',
            'elevator_outer_2_joint',
            'top_gripper_right_arm_joint',
            'top_gripper_left_arm_joint',
            'top_slider_joint',
            'bottom_intake_joint',
            'bottom_gripper_right_arm_joint',
            'bottom_gripper_left_arm_joint',
        ]   

    def get_action(self, msg):
        obs = np.array([msg.data], dtype=np.float32)
        action = self.policy(torch.tensor(obs).float())
        self.twist_msg.linear.x = action[0].detach().numpy()
        self.twist_msg.linear.y = action[1].detach().numpy()
        self.twist_msg.angular.z = action[2].detach().numpy()
        self.position_cmds.positions = [
            action[3].detach().numpy(),
            action[4].detach().numpy(),
            action[5].detach().numpy(),
            action[4].detach().numpy(),
            action[6].detach().numpy(),
            action[6].detach().numpy(),
            action[7].detach().numpy(),
            action[6].detach().numpy(),
            action[8].detach().numpy(),
            action[8].detach().numpy(),
        ]
        self.cmds.joint_names = self.joints
        self.cmds.points = [self.position_cmds]
        
        self.publisher_.publish(self.cmds)
        self.action_pub.publish(self.twist_msg)
        self.step += 1
    def joint_state_callback(self, msg):
        if(msg != None):
            self.joint_state_msg = msg
        return
    def odom_callback(self, msg):
        if(msg != None):
            self.odom_msg = msg
        return

    def get_reward():
        return
def main(args=None):
    # env = gym.create_env("RealRobot", ip=self.robot_ip)
    rclpy.init(args=args)
    runner = Runner()
    rclpy.spin(runner)
    # env.disconnect()
if __name__ == '__main__':
    main()