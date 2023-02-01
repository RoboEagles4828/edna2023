import rclpy
from rclpy.node import Node
import math
import time

from sensor_msgs.msg import JointState
vel_cmds = JointState() 
rad = math.pi
vel_cmds.velocity = [0.0, 0.0, 0.0, 0.0, rad, rad, rad, rad]
vel_cmds.position = [0.0, 0.0, 0.0, 0.0, rad, rad, rad, rad]
vel_cmds.name = [
        'front_left_wheel_joint',
        'front_left_axle_joint',
        'front_right_wheel_joint',
        'front_right_axle_joint',
        'rear_left_wheel_joint',
        'rear_left_axle_joint',
        'rear_right_wheel_joint',
        'rear_right_axle_joint']
        
class RunTests(Node):
    def __init__(self):
        super().__init__('run_tests')
        # publisb the test commands
        self.publisher_ = self.create_publisher(JointState, 'isaac_joint_commands', 10)  
        self.publisher_.publish(vel_cmds)
        self.get_logger().info('Testing: ...')
        # Wait a littlt bit
        time.sleep(1)
        # Read the subscriber
        my_callback_groups = None
        self.subscription = self.create_subscription(JointState, 'isaac_joint_states', qos_profile=10, callback=my_callback_groups)
        self.subscription 
        # Compare the values from subscriber to commands
def main(args=None):
    rclpy.init(args=args)

    node = RunTests()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()