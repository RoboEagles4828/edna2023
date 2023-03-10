from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import networktables



# create reader instance and open for reading
with Reader('auto_ros_bag') as reader:
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/cmd_vel':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            print(msg.header.frame_id)