import time
from rclpy.node import Node
from sensor_msgs.msg import JointState

# COLORS
GREEN = "\033[0;32m"
YELLOW = "\033[1;33m"
RED = "\033[0;31m"
RESET = "\033[0m"

# Default ROS values
SUBSCRIBE_TOPIC_NAME = '/real/real_joint_states'
PUBLISH_INTERVAL = .5 # seconds

# Default Tolerances
PASS_TOLERANCE = 0.5
WARN_TOLERANCE = 1

SCALING_FACTOR_FIX = 10000

class TesterNode(Node):

    def __init__(self, tests, joint_names, joint_range, pub_topic_name, sub_topic_name=SUBSCRIBE_TOPIC_NAME, pub_interval=PUBLISH_INTERVAL, pass_tolerance=PASS_TOLERANCE, warn_tolerance=WARN_TOLERANCE):
        super().__init__('arm_tester')
        # Constructor Arguments
        self.TESTS = tests
        self.JOINT_NAMES = joint_names
        self.JOINT_RANGE = joint_range
        self.SUBSCRIBE_TOPIC_NAME = sub_topic_name 
        self.PUBLISH_TOPIC_NAME = pub_topic_name 
        self.PUBLISH_INTERVAL = pub_interval
        self.PASS_TOLERANCE = pass_tolerance
        self.WARN_TOLERANCE = warn_tolerance

        self.currentTest = 0
        self.testStatus = []
        self.lastPositions = [None]*6
        self.expectedPositions = self.TESTS[self.currentTest]["positions"]
        self.recieving = True
        self.startTime = time.time()

        self.subscription = self.create_subscription(JointState, self.SUBSCRIBE_TOPIC_NAME, self.recieve, 10)
        self.publisher = self.create_publisher(JointState, self.PUBLISH_TOPIC_NAME, 10)
        self.publishTimer = self.create_timer(self.PUBLISH_INTERVAL, self.publish)

    def publish(self):
        msg = JointState()
        msg.name = self.JOINT_NAMES
        msg.position = self.TESTS[self.currentTest]["positions"]
        self.publisher.publish(msg)
        self.doTests()

    def doTests(self):
        if not(self.recieving):
            # Start running the next test
            self.currentTest += 1
            if self.currentTest > len(self.TESTS)-1:
                print("\nTests Finished")
                for index, test in enumerate(self.testStatus):
                    print(f"{RED if test['fail'] > 0 else YELLOW if test['warn'] > 0 else GREEN}Test {index} {'FAILED' if test['fail'] > 0 else 'PASSED'} {'with WARNINGS' if test['warn'] > 0 else ''}{RESET}")
                exit(0) # shutting down rclpy just kills the node and hangs the process, without actually stopping the program
            else:
                self.expectedPositions = self.TESTS[self.currentTest]["positions"]
                self.lastPositions = [None]*6
                self.recieving = True
                self.startTime = time.time()
        else:
            # Check if the current test should end...
            timeleft = time.time() - self.startTime
            print(f"\rRunning Test {self.currentTest} ({round(self.TESTS[self.currentTest]['time'] - timeleft, 2)}s remaining)", end='')
            if time.time() - self.startTime > self.TESTS[self.currentTest]["time"]:
                self.recieving = False
                self.testStatus.append({"pass": 0, "warn": 0, "fail": 0})
                print(f"\rTest {self.currentTest} Completed                                       ")
                self.printResults()
                print()

    def recieve(self, msg : JointState):
        if self.recieving:
            self.lastPositions = [i / SCALING_FACTOR_FIX for i in msg.position[self.JOINT_RANGE[0]:self.JOINT_RANGE[1]]]

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
            elif difference <= self.PASS_TOLERANCE:
                print(f"{GREEN}Difference of {difference} (expected {self.expectedPositions[index]}, got {position}){RESET}")
                self.testStatus[self.currentTest]["pass"] += 1
            elif difference <= self.WARN_TOLERANCE:
                print(f"{YELLOW}Difference of {difference} (expected {self.expectedPositions[index]}, got {position}){RESET}")
                self.testStatus[self.currentTest]["warn"] += 1
            else:
                print(f"{RED}FAILED: Expected position {self.expectedPositions[index]}, got {position}{RESET}")
                self.testStatus[self.currentTest]["fail"] += 1