# ROS2 Wrapper for ZED2i SDK - Publisher & Subscriber

This package provides a ROS2 wrapper for the Zedi2 SDK. It allows to publish the Zedi2 SDK data/frames as ROS2 messages, and enables any connected device with the subscriber interface to receive the data/frames.

## Pre-requisites
- Ubuntu 20.04
- ROS2 Foxy
- OpenCV


## Running the ZEDi2 ROS2 Wrapper
```bash
source /opt/ros/foxy/setup.bash
```

```bash
cd ~/zed_opencv_pi/src/
colcon build --symlink-install
. install/setup.bash
```
### Run publisher
```bash
ros2 run zed_cv zed_sub
```

### Run subscriber
```bash
ros2 run zed_cv zed_pub
```
