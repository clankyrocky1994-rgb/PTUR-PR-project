# Robot Welding Trajectory Project

This project contains a ROS 2 package for planning and visualizing a simple welding-like Cartesian trajectory for a KUKA LBR robot.

The trajectory is defined by XYZ points, converted to joint-space using MoveIt IK, visualized in RViz, and optionally executed through `joint_trajectory_controller`.

## Environment

Tested with:

* Ubuntu 24.04
* ROS 2 Jazzy
* Gazebo
* RViz
* MoveIt
* lbr_fri_ros2_stack
* Robot model: `iiwa7`

## Repository structure

```bash
project/
├── README.md
├── .gitignore
└── src/
    └── my_robot_control/
```

## 1. Install ROS 2 Jazzy

Install ROS 2 Jazzy Desktop for Ubuntu 24.04.

After installation, source ROS:

```bash
source /opt/ros/jazzy/setup.bash
```

Optional:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 2. Install ROS tools

```bash
sudo apt update
sudo apt install ros-dev-tools python3-colcon-common-extensions python3-vcstool python3-rosdep -y
```

If `rosdep` is not initialized:

```bash
sudo rosdep init
rosdep update
```

## 3. Install and build lbr_fri_ros2_stack

Create a workspace for the LBR stack:

```bash
mkdir -p ~/prj/lbr-stack/src
cd ~/prj/lbr-stack
source /opt/ros/jazzy/setup.bash
```

Clone the LBR stack:

```bash
git clone https://github.com/lbr-stack/lbr_fri_ros2_stack.git -b jazzy src/lbr_fri_ros2_stack
```

Import dependencies:

```bash
export FRI_CLIENT_VERSION=1.15
vcs import src < src/lbr_fri_ros2_stack/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml
```

Install dependencies:

```bash
rosdep install --from-paths src -i -r -y
```

Build:

```bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Clone this project

```bash
cd ~
git clone https://github.com/clankyrocky1994-rgb/test.git project
cd ~/project
```

Source ROS and the LBR stack:

```bash
source /opt/ros/jazzy/setup.bash
source ~/prj/lbr-stack/install/setup.bash
```

Install dependencies for this project:

```bash
rosdep install --from-paths src -i -r -y
```

Build this project:

```bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Run Gazebo

Terminal 1:

```bash
cd ~/prj/lbr-stack
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lbr_bringup gazebo.launch.py model:=iiwa7
```

## 6. Run MoveIt and RViz

Terminal 2:

```bash
cd ~/prj/lbr-stack
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lbr_bringup move_group.launch.py model:=iiwa7 mode:=gazebo rviz:=true
```

## 7. Visualize the welding trajectory

Terminal 3:

```bash
cd ~/project
source /opt/ros/jazzy/setup.bash
source ~/prj/lbr-stack/install/setup.bash
source install/setup.bash
ros2 run my_robot_control welding_line_ik
```

In RViz, add the marker topics:

```text
Add → By topic → /welding_planned_line → Marker
Add → By topic → /welding_points → Marker
```

## 8. Execute the trajectory

Before execution, check that the trajectory controller action exists:

```bash
ros2 action list | grep trajectory
```

Expected:

```text
/lbr/joint_trajectory_controller/follow_joint_trajectory
```

Run the trajectory:

```bash
cd ~/project
source /opt/ros/jazzy/setup.bash
source ~/prj/lbr-stack/install/setup.bash
source install/setup.bash
ros2 run my_robot_control welding_line_ik --ros-args -p execute:=true
```

## Main file

The main script is:

```bash
src/my_robot_control/my_robot_control/welding_line_ik.py
```

Important parameters:

```python
self.start_xyz = [-0.45, -0.20, 0.45]
self.end_xyz = [-0.45, 0.20, 0.45]
self.num_points = 40
self.total_time_sec = 12.0
```

Tool orientation is configured in the same file through the orientation settings / quaternion logic.

## Notes

* `execute:=false` is the default safety mode. It only calculates IK and visualizes the planned trajectory.
* `execute:=true` sends the trajectory to the robot controller.
* Always test in Gazebo before running on real hardware.
* Do not commit `build/`, `install/`, or `log/`.
