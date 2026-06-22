# Robot Vision Welding Replanner

ROS 2 Jazzy project for KUKA LBR / iiwa7 welding trajectory visualization and live obstacle-aware replanning using hand detection from a camera.

The system connects:

* KUKA LBR / iiwa7 simulation in Gazebo
* MoveIt IK service
* ROS 2 hand detection bridge
* live welding trajectory replanner
* RViz visualization of hand position and replanned path

## Workspace

Main ROS 2 workspace:

```bash
/home/daun/project
```

KUKA LBR stack workspace:

```bash
/home/daun/dep/lbr-stack
```

Robot vision app:

```bash
/home/daun/project/robot_vision_app
```

## Packages

This repository contains:

```text
src/my_robot_control
src/robot_vision_msgs
src/robot_vision_ros2
robot_vision_app
```

Main nodes:

```text
my_robot_control/welding_replanner
my_robot_control/live_welding_replanner
robot_vision_ros2/hand_bridge.launch.py
```

At the moment, `welding_replanner` entry point runs the live welding replanner.

## Build

```bash
cd /home/daun/project
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
```

For rebuilding only the control package:

```bash
cd /home/daun/project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-select my_robot_control
source install/setup.bash
```

## Full Run Procedure

### 1. Start KUKA / Gazebo / MoveIt

In terminal 1:

```bash
cd /home/daun/dep/lbr-stack
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lbr_bringup gazebo.launch.py ctrl:=joint_trajectory_controller model:=iiwa7
```

### 2. Start fake camera TF

In terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source /home/daun/project/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x -0.8 \
  --y 0.0 \
  --z 0.7 \
  --roll 3.1416 \
  --pitch 0.0 \
  --yaw 3.1416 \
  --frame-id lbr_link_0 \
  --child-frame-id fake_camera_link
```

Alternative transform if the Z axis looks inverted:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x -0.8 \
  --y 0.0 \
  --z 0.7 \
  --roll 0.0 \
  --pitch 3.1416 \
  --yaw 3.1416 \
  --frame-id lbr_link_0 \
  --child-frame-id fake_camera_link
```

### 3. Start robot vision ROS 2 bridge

In terminal 3:

```bash
cd /home/daun/project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot_vision_ros2 hand_bridge.launch.py frame_id:=fake_camera_link
```

### 4. Start camera hand detection app

In terminal 4:

```bash
cd /home/daun/project/robot_vision_app
source /home/daun/project/vision_venv/bin/activate

python src/robot_vision_v3.py --config config/config.yaml
```

### 5. Start live welding replanner

In terminal 5:

```bash
cd /home/daun/project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run my_robot_control welding_replanner
```

Expected output:

```text
live_welding_replanner started
Waiting for robot data...
Sending initial welding trajectory
Trajectory accepted
HAND TOO CLOSE
Cancel requested
Sending new live-replanned trajectory
Trajectory accepted
```

## RViz

Use fixed frame:

```text
lbr_link_0
```

Useful displays:

```text
TF
/live_hand_marker
/live_replanned_path
```

Avoid using old marker topics at the same time, especially:

```text
/hand_obstacle_marker
/robot_vision/markers
```

Otherwise two hand markers may appear in different positions.

## Important Topics

Hand input:

```text
/robot_vision/hands
```

Live hand marker:

```text
/live_hand_marker
```

Live replanned path:

```text
/live_replanned_path
```

Robot controller action:

```text
/lbr/joint_trajectory_controller/follow_joint_trajectory
```

MoveIt IK service:

```text
/lbr/compute_ik
```

## Debug Commands

Check ROS topics:

```bash
ros2 topic list | grep -E "robot_vision|live|hand"
```

Check controller status:

```bash
ros2 control list_controllers
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo lbr_link_0 fake_camera_link
```

Check MoveIt IK service:

```bash
ros2 service list | grep compute_ik
```

Check trajectory action:

```bash
ros2 action list | grep trajectory
```

## Current Replanning Logic

The live replanner works as follows:

1. The robot starts moving from welding start point A to end point B.
2. The node reads the detected hand position from `/robot_vision/hands`.
3. The hand position is transformed from `fake_camera_link` to `lbr_link_0`.
4. If the hand is too close to the current welding path, the active trajectory is cancelled.
5. A new detour point C is created.
6. A new trajectory A → C → B is sent to the robot controller.

This is reactive live replanning with a detour point. It is not a full potential field planner.

## Notes

Generated folders are ignored by git:

```text
build/
install/
log/
vision_venv/
```

Large model files and datasets should not be committed directly. Use release assets, external storage, or Git LFS if needed.
