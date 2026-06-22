# Robot Vision Live Welding Replanner

ROS 2 Jazzy project for live welding trajectory replanning with KUKA LBR / iiwa7, MoveIt, Gazebo, RViz and camera-based hand detection.

The robot follows an initial welding path. If a hand is detected too close to the path, the active trajectory is cancelled and a new avoidance trajectory is sent to the robot controller.

## Main Components

* KUKA LBR / iiwa7 simulation
* MoveIt IK service
* ROS 2 hand detection bridge
* camera-based hand detection app
* live welding trajectory replanner
* RViz visualization

## Repository Structure

```text
src/my_robot_control
src/robot_vision_msgs
src/robot_vision_ros2
robot_vision_app
```

## Requirements

This project uses:

* Ubuntu 24.04
* ROS 2 Jazzy
* KUKA LBR ROS 2 stack
* Gazebo
* MoveIt
* Python 3
* OpenCV / MediaPipe / Ultralytics for vision

The Python dependencies for the camera detection app are listed in:

```text
robot_vision_app/requirements.txt
```

## Build ROS 2 Workspace

From the root of this repository:

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
```

Rebuild only the control package:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-select my_robot_control
source install/setup.bash
```

## Install Python Dependencies for Vision App

Run this from the root of this repository:

```bash
python3 -m venv vision_venv
source vision_venv/bin/activate

pip install --upgrade pip
pip install -r robot_vision_app/requirements.txt
```

The virtual environment folder should not be committed to git:

```text
vision_venv/
```

## Run

Open separate terminals for each step.

## 1. Start KUKA / Gazebo / MoveIt

Run this from the KUKA LBR stack workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lbr_bringup gazebo.launch.py ctrl:=joint_trajectory_controller model:=iiwa7
```

## 2. Start Fake Camera TF

Run this from the root of this repository:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

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
source /opt/ros/jazzy/setup.bash
source install/setup.bash

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

## 3. Start Robot Vision ROS 2 Bridge

From the root of this repository:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot_vision_ros2 hand_bridge.launch.py frame_id:=fake_camera_link
```

This bridge publishes detected hand data to ROS 2.

Main topic:

```text
/robot_vision/hands
```

## 4. Start Camera Hand Detection App

From the root of this repository:

```bash
cd robot_vision_app
source ../vision_venv/bin/activate

python src/robot_vision_v3.py --config config/config.yaml
```

If dependencies are not installed yet, run this first from the repository root:

```bash
python3 -m venv vision_venv
source vision_venv/bin/activate

pip install --upgrade pip
pip install -r robot_vision_app/requirements.txt
```

## 5. Start Live Welding Replanner

From the root of this repository:

```bash
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

Do not enable old marker topics at the same time:

```text
/hand_obstacle_marker
/robot_vision/markers
```

Otherwise two hand markers may appear.

## Useful Commands

Check hand and replanning topics:

```bash
ros2 topic list | grep -E "robot_vision|live|hand"
```

Check robot controllers:

```bash
ros2 control list_controllers
```

Expected active controllers include:

```text
joint_state_broadcaster
joint_trajectory_controller
```

Check fake camera TF:

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

Check hand messages:

```bash
ros2 topic echo /robot_vision/hands
```

## Replanning Logic

1. The robot starts moving from welding start point A to end point B.
2. The node reads hand position from `/robot_vision/hands`.
3. The hand position is transformed from `fake_camera_link` to `lbr_link_0`.
4. If the hand is too close to the welding path, the current trajectory is cancelled.
5. A detour point C is generated.
6. A new trajectory A → C → B is sent to the robot controller.

This is reactive live replanning with a detour point. It is not a full potential field planner.

## Important Notes

The live replanner currently uses:

```text
/robot_vision/hands
/live_hand_marker
/live_replanned_path
/lbr/compute_ik
/lbr/joint_trajectory_controller/follow_joint_trajectory
```

The old obstacle marker node should not be launched together with the live replanner, otherwise duplicate hand markers may appear in RViz.

## Git Notes

Do not commit generated or local files:

```text
build/
install/
log/
vision_venv/
*.zip
*.bag
*.db3
*.mcap
```

Large model files should either be ignored or stored with Git LFS.

If model weights are stored with Git LFS:

```bash
git lfs install
git lfs track "robot_vision_app/models/*.pt"
git lfs track "robot_vision_app/models/*.pth"
git lfs track "robot_vision_app/models/*.onnx"
git lfs track "robot_vision_app/models/*.engine"
```

Then add:

```bash
git add .gitattributes
git add -f robot_vision_app/models
```

