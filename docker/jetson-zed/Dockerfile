FROM arm64v8/ros:humble-perception AS ros2
FROM stereolabs/zed:3.8-tools-devel-jetson-jp5.0.2 AS zed-jetson
WORKDIR /root/
COPY --from=ros2 /opt/ros/humble /opt/ros/humble