import rclpy
from rclpy.node import Node
import math
import time
from rclpy.action import ActionClient
from action_tutorials_interfaces.action import Fibonacci
from sensor_msgs.msg import JointState
import logging
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

test_return = JointState()
        
class RunTests(Node):
    def __init__(self):
        super().__init__('run_tests')
        # publisb the test commands
        self.publisher_ = self.create_publisher(
            JointState, 
            'real_joint_commands', 
            10
        )  
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription = self.create_subscription(
            JointState,
            'real_joint_states',
            self.listener_callback,
            10
        )
        self.subscription
        self.get_logger().info('Testing: ...')
    def timer_callback(self):
        self.publisher_.publish(vel_cmds)
        time.sleep(0.1)
        self.i+=1
    def listener_callback(self, msg):
        test_return = msg
def check(msg, test, test_fail):
        count = 0
        working = True
        for x in msg.velocity:
            if (abs(msg.velocity[count] / 10000 - vel_cmds.velocity[count]) > 0.1 or abs(msg.position[count] / 10000 - vel_cmds.position[count]) > 0.1):
                working = False
                count+=1
        if not working:
            print(test)
        else:
            print(test_fail)
def timeLoop(time):
    
def test1(node):
    vel_cmds.velocity=[2.0, 0.0, 0.0, 0.0, rad, rad, rad, rad]
    t_end = time.time() + 10
    while time.time() < t_end:
        rclpy.spin_once(node)
        time.sleep(1)
    check(test_return, 'Test Passed: Front Left Wheel is Spinning!', 'ERROR: Front Left Wheel is NOT spinning, something is wrong!')
def test2(node):
    vel_cmds.velocity=[0.0, 2.0, 0.0, 0.0, rad, rad, rad, rad]
    t_end = time.time() + 10
    while time.time() < t_end:
        rclpy.spin_once(node)
        time.sleep(1)
    check(test_return, 'Test Passed: Front Left Axle is Spinning!', 'ERROR: Front Left Axle is NOT spinning, something is wrong!')
def test3(node):
    vel_cmds.velocity=[0.0, 0.0, 2.0, 0.0, rad, rad, rad, rad]
    t_end = time.time() + 10
    while time.time() < t_end:
        rclpy.spin_once(node)
        time.sleep(1)
    check(test_return, 'Test Passed: Front Right Wheel is Spinning!', 'ERROR: Front Right Wheel is NOT spinning, something is wrong!')
def test4(node):
    vel_cmds.velocity=[0.0, 0.0, 0.0, 2.0, rad, rad, rad, rad]
    t_end = time.time() + 10
    while time.time() < t_end:
        rclpy.spin_once(node)
        time.sleep(1)
    check(test_return, 'Test Passed: Front Right Axle is Spinning!', 'ERROR: Front Right Axle is NOT spinning, something is wrong!')
def testAll(node):
    # setup tests commands
    # go into while loop that will track the time 
    # spin and sleep inside the while loop
    # 
def main(args=None):
    rclpy.init(args=args)

    node = RunTests()

    rclpy.spin_once(node)

    test1(node)

    test2(node)

    test3(node)

    test4(node)

    node.destroy_node()

    rclpy.shutdown()
if __name__ == "__main__":
    main()