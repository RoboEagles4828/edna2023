# Setting up the ZED Camera with ROS2 Galactic

## Chapter One: Downloading and Installing the Packages
This assumes that you already have a working install of [ROS2 Galactic](https://docs.ros.org/en/galactic/index.html) and the [ZED SDK](https://www.stereolabs.com/developers/release/). You're going to need to go to the [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) and follow the installation insructions to add it here. However, before running `rosdep install`, you'll need to go into the src/ folder and move all of the packages inside of the zed-ros2-wrapper folder into the root src folder.

**WARNING: INCREDIBLY HACKY FIX AHEAD**

*If you're having problems with anything, this is probably the cause of it*

If you try to `colcon build` at this point, you're going to run into some strange error having to do with install/zed_components/lib/libzed_camera_component.so regarding tf2::doTransform. Now, if you're willing to look into why this is erroring, please feel free to fix the code, but if you want a fast and simple solution, then just do this.
Go into src/zed_components/src/zed_camera/src/zed_camera_component.cpp, and scroll down to line 6333. Comment that line out, save the file, and continue on with the rest of this.

Once you've done that, you're also going to need the [ZED ROS2 Examples](https://github.com/stereolabs/zed-ros2-examples) for the 3D bounding boxes to display in RViz2. Go ahead and download their repository, making sure to take the packages out of the folder and put it in src/ directly.


With all of that done, you can finally run `colcon build`.

## Chapter Two: Running RViz2

Now that everything is set up and you ran into no errors whatsoever, you can run it!

Running `ros2 launch zed_wrapper zed2i.launch.py` (make sure to source ros2 and the local install at install/local_setup.bash first) will start a ROS topic publishing out the camera information!

You can also run `ros2 launch zed_display_rviz2 display_zed2i.launch.py` to open RViz2 with the default ZED configuration (useful for the next step).

## Chapter Three: Ordinary Object Detection

If you just want to enable object detection temporarily, then you can run `ros2 service call /zed2i/zed_node/enable_obj_det std_stvs/srv/SetBool data:\ true`. This command will enable object detection for the currently running ROS node.

However, if you want object detection to be on by default, then go into src/zed_wrapper/config/common.yaml, and change line 107 to be true.
(You can also mess with the other object detection settings here).

Now if you open RViz2 with the default ZED settings, you can see the bounding boxes going around the real world things! (You can also add the point cloud from the topic to see it even better).

## Chapter Four: Custom Object Detection
Unfortunately, custom object detection is *not* currently included in the ROS2 wrapper for the ZED SDK. This could be done by modifying the `zed_wrapper` package manually to add in a supported detection algorithm like Yolo or OpenCV. For us, we ran out of time to get this working during the season, but the above steps can give you a starting point. For more information, head over to [the ZED SDK](https://github.com/stereolabs/zed-sdk/tree/master/object%20detection/custom%20detector/cpp/tensorrt_yolov5_v6.0) and look at their code.