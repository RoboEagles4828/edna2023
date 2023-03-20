FROM nvcr.io/nvidia/isaac-sim:2022.2.1

# Set ROS2 Humble as the default ROS Bridge
RUN sed -i 's/ros_bridge/ros2_bridge-humble/' /isaac-sim/apps/omni.isaac.sim.base.kit

# Add allow root to warm up
RUN echo " --allow-root" >> /isaac-sim/omni.isaac.sim.warmup.sh