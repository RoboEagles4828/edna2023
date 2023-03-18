import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState


class PublishJointCmd(Node):

    def __init__(self):
        super().__init__('publish_arm_joint_commands')
        self.publisher_ = self.create_publisher(JointState, '/real/real_arm_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # velocity_cmds = JointState()
        position_cmds = JointState()
        
        position_cmds.name = [
            # Pneumatics
            'arm_roller_bar_joint',
            'top_slider_joint',
            'top_gripper_left_arm_joint',
            # Wheels
            'elevator_center_joint',
            'bottom_intake_joint',
        ]
        # position_cmds.name = []
        # rad = math.pi
        # velocity_cmds.velocity = [ 0.0 ] * 8
        position_cmds.position = [ 
            0.0,      # Either a 0 (down) or a 1 (up) 
            0.0,      # Either a 0 (fully back) or a 1 (fully extended)
            0.0,      # Either a 0 (open) or a 1 (closed)
            0.0,      # Value between 0.0 (fully back) and 2.0 (fully extended) (will be converted on their end, so just take the motor value and multiply it by two)
            0.0       # Value between 0.0 (fully down) and 1.0 (fully up)
        ]
        # position_cmds.position = []

        self.publisher_.publish(position_cmds)
        # self.publisher_.publish(position_cmds)
        self.get_logger().info('Publishing: ...')
        self.i += 1




def main(args=None):
    rclpy.init(args=args)

    node = PublishJointCmd()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()