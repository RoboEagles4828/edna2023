import rclpy
import time

from rclpy.node import Node
from sensor_msgs.msg import JointState

TESTS = [
    # Here's where you define your tests, in the same style as this one.
    {"positions": [0.0]*6, "time": 3.0},

]


# ROS Topics
SUBSCRIBE_TOPIC_NAME = '/real/real_joint_states'
PUBLISH_TOPIC_NAME = '/real/real_arm_commands'
PUBLISH_INTERVAL = .5 # seconds

# Tolerances
PASS_TOLERANCE = 0.5
WARN_TOLERANCE = 1

# COLORS
GREEN = "\033[0;32m"
YELLOW = "\033[1;33m"
RED = "\033[0;31m"
RESET = "\033[0m"

class TesterNode(Node):

    def __init__(self):
        super().__init__('arm_tester')
        self.currentTest = 0
        self.testStatus = []
        self.lastPositions = [None]*6
        self.expectedPositions = TESTS[self.currentTest]["positions"]
        self.recieving = True
        self.startTime = time.time()

        self.subscription = self.create_subscription(JointState, SUBSCRIBE_TOPIC_NAME, self.recieve, 10)
        self.publisher = self.create_publisher(JointState, PUBLISH_TOPIC_NAME, 10)
        self.publishTimer = self.create_timer(PUBLISH_INTERVAL, self.publish)

    def publish(self):
        msg = JointState()
        msg.name = [
            # Pneumatics
            'arm_roller_bar_joint', 
            'top_gripper_slider_joint',     # Not in URDF yet
            'top_gripper_joint',            # Not in URDF yet
            'bottom_gripper_joint',         # Not in URDF yet
            # Wheels
            'elevator_left_elevator_center_joint',
            'bottom_gripper_lift_joint' 
        ]
        msg.position = TESTS[self.currentTest]["positions"]
        self.publisher.publish(msg)
        self.doTests()

    def doTests(self):
        if not(self.recieving):
            # Start running the next test
            self.currentTest += 1
            if self.currentTest > len(TESTS)-1:
                print("\nTests Finished")
                for index, test in enumerate(self.testStatus):
                    print(f"{RED if test['fail'] > 0 else YELLOW if test['warn'] > 0 else GREEN}Test {index} {'FAILED' if test['fail'] > 0 else 'PASSED'} {'with WARNINGS' if test['warn'] > 0 else ''}{RESET}")
                exit(0) # shutting down rclpy just kills the node and hangs the process, without actually stopping the program
            else:
                self.expectedPositions = TESTS[self.currentTest]["positions"]
                self.lastPositions = [None]*6
                self.recieving = True
                self.startTime = time.time()
        else:
            # Check if the current test should end...
            if time.time() - self.startTime > TESTS[self.currentTest]["time"]:
                self.recieving = False
                self.testStatus.append({"pass": 0, "warn": 0, "fail": 0})
                print(f"\nTest {self.currentTest} Completed")
                self.printResults()

    def recieve(self, msg : JointState):
        if self.recieving:
            self.lastPositions = msg.position[8:]

    def testFinished(self):
        if not self.recieving:
            return
        self.recieving = False
        self.destroy_timer(self.publishTimer)
        self.i += 1
            
    def printResults(self):
        for index, position in enumerate(self.lastPositions):
            if position is None:
                print(f"{RED}FAILED: Did not recieve any positions from the robot!{RESET}")
                self.testStatus[self.currentTest]["fail"] += 1
                continue
            difference = abs(self.expectedPositions[index] - position)
            if difference == 0:
                print(f"{GREEN}Joint {index} PASSED{RESET}")
                self.testStatus[self.currentTest]["pass"] += 1
            elif difference <= PASS_TOLERANCE:
                print(f"{GREEN}Difference of {difference} (expected {self.expectedPositions[index]}, got {position}){RESET}")
                self.testStatus[self.currentTest]["pass"] += 1
            elif difference <= WARN_TOLERANCE:
                print(f"{YELLOW}Difference of {difference} (expected {self.expectedPositions[index]}, got {position}){RESET}")
                self.testStatus[self.currentTest]["warn"] += 1
            else:
                print(f"{RED}FAILED: Expected position {self.expectedPositions[index]}, got {position}{RESET}")
                self.testStatus[self.currentTest]["fail"] += 1

def main():
    rclpy.init()
    testerNode = TesterNode()
    rclpy.spin(testerNode)

if __name__ == '__main__':
    main()