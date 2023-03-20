import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState


class PublishJointCmd(Node):

    def __init__(self):
        super().__init__('publish_drive_joint_commands')
        self.publisher_ = self.create_publisher(JointState, '/real/real_joint_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        cmds = JointState()
        cmds.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            
            'front_left_axle_joint',
            'front_right_axle_joint',
            'rear_left_axle_joint',
            'rear_right_axle_joint'
        ]
        rad = math.pi
        cmds.velocity = [ 
            0.0,
            0.0,
            0.0,
            0.0,
            0.0, #ignore
            0.0, #ignore
            0.0, #ignore
            0.0, #ignore
        ]
        cmds.position = [
            0.0, #ignore
            0.0, #ignore
            0.0, #ignore
            0.0, #ignore
            rad,
            0.0,
            0.0,
            0.0
        ]
        # position_cmds.position = []

        self.publisher_.publish(cmds)
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