from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class StageSubscriber(Node):

    def __init__(self):
        super().__init__('stage_subscriber')
        self.subscription = self.create_subscription(
            String,
            'frc_stage',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if msg.data == "AUTON":
            with Reader('auto_ros_bag') as reader:
                # topic and msgtype information is available on .connections list
                for connection in reader.connections:
                    print(connection.topic, connection.msgtype)

                # iterate over messages
                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == '/cmd_vel':
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        print(msg.header.frame_id)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# create reader instance and open for reading
