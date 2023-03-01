import rclpy
from rclpy.node import Node
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PublishTrajectoryMsg(Node):

    def __init__(self):
        super().__init__('publish_trajectory_msg')

        # self.declare_parameters(
        #     namespace = NAMESPACE
        #     parameters = [{
        #         "arm_roller_bar_joint": f"{namespace}_arm_roller_bar_joint",
        #         "elevator_left_elevator_outer_1_joint": f"{namespace}_elevator_left_elevator_outer_1_joint",
        #         "elevator_left_elevator_outer_2_joint": f"{namespace}_elevator_left_elevator_outer_2_joint",
        #         "elevator_left_elevator_center_joint": f"{namespace}_elevator_left_elevator_center_joint",
        #         "elevator_right_elevator_outer_1_joint": f"{namespace}_elevator_right_elevator_outer_1_joint",
        #         "elevator_right_elevator_outer_2_joint": f"{namespace}_elevator_right_elevator_outer_1_joint",
        #         "elevator_right_elevator_center_joint": f"{namespace}_elevator_right_elevator_outer_1_joint",
        #         "elevator_left_elevator_leg_joint": f"{namespace}_elevator_left_elevator_leg_joint",
        #         "elevator_right_elevator_leg_joint": f"{namespace}_elevator_right_elevator_leg_joint",
        #     }]
        # )

        self.publisher_ = self.create_publisher(JointTrajectory, 'trajectory_joint_msgs', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        velocity_cmds = JointTrajectory()
        # position_cmds = JointState()
        
        velocity_cmds.joint_names = [
            ''
            'arm_roller_bar_joint',
            'elevator_left_elevator_center_joint']
        # position_cmds.name = []
        rad = math.pi
        # velocity_cmds.velocity = [ 0.0 ] * 8
        stuff = JointTrajectoryPoint()
        stuff.velocities = [0.0, 0.0]
        velocity_cmds.points = [stuff]
        # position_cmds.position = []

        self.publisher_.publish(velocity_cmds)
        # self.publisher_.publish(position_cmds)
        self.get_logger().info('Publishing: ...')
        self.i += 1


# 'front_left_wheel_joint',
#             'front_left_axle_joint',
#             'front_right_wheel_joint',
#             'front_right_axle_joint',
#             'rear_left_wheel_joint',
#             'rear_left_axle_joint',
#             'rear_right_wheel_joint',
#             'rear_right_axle_joint']

def main(args=None):
    rclpy.init(args=args)

    node = PublishTrajectoryMsg()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()