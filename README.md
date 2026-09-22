# hex_ros_demo_joystick
[中文](README_cn.md) | **English**

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Topics](#topics)
- [Joystick Mapping](#joystick-mapping)
- [Parameters](#parameters)
- [Project Structure](#project-structure)

## Overview

`hex_ros_demo_joystick` demonstrates joystick teleoperation for an Archer Y6. It reads a Linux joystick, converts end-effector translation, rotation, and gripper input into arm commands, and executes the commands in MuJoCo with target visualization.

It provides:

- one launch for MuJoCo, joystick input, arm control, and RViz;
- Cartesian position and orientation control;
- gripper, reset, and exit button mappings.

The current launch target is MuJoCo simulation. This package supports **ROS 2 Humble** and is compatible with **ROS 1 Noetic**.

## Quick Start

> Complete [Installation](#installation) before launching.

### 1. Launch the Simulation

`sim_joy_arm` starts MuJoCo, joystick input, `joy_arm`, and the RViz configuration owned by this package. The parameter and RViz files are:

```text
config/<ros_version>/joy_arm.yaml
config/<ros_version>/joy_arm.rviz
```

The joystick input node reads `device_path` from the `hex_ros_teleop_joystick` parameter file. Its empty default auto-detects the first joystick, so launch directly first:

**ROS 2:**

```shell
ros2 launch hex_ros_demo_joystick sim_joy_arm.launch.py viewer:=true rviz:=true
```

**ROS 1:**

```shell
roslaunch hex_ros_demo_joystick sim_joy_arm.launch viewer:=true rviz:=true
```

If auto-detection fails or multiple input devices are present, list stable device paths:

```shell
ls -l /dev/input/by-id/
```

Then set `device_path` in the applicable downstream parameter file in the source workspace:

```text
# ROS 2
<your_ws>/src/hex_ros_teleop_joystick/config/ros2/params.yaml

# ROS 1
<your_ws>/src/hex_ros_teleop_joystick/config/ros1/params.yaml
```

For example:

```yaml
device_path: "/dev/input/by-id/<joystick-device>"
```

For ROS 2, keep the parameter under `/**/teleop_joystick.ros__parameters`. Rebuild and source the workspace after changing it.

You may also use `/dev/input/eventX`; determine `X` with `ls -l /dev/input/` or from the `by-id` symlink target. An incorrect path prevents the joystick input node from operating correctly.

If the path exists but cannot be opened, check the current user's read permission and device-group membership; do not bypass device access control with overly broad permissions. If the terminal prints `no joystick device found`, the launch may remain running but no valid joystick state is published. Fix the path or permissions first, then verify input with `ros2 topic echo /teleop_joy_state` (ROS 1: `rostopic echo /teleop_joy_state`).

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `viewer` | `true` | Start the MuJoCo viewer. |
| `rviz` | `true` | Start the RViz instance owned by this package. |
| `use_sim_time` | `true` | Use simulation time. |

After launch, the simulated arm first moves automatically to `stable_joint`. Normal joystick control begins after the terminal prints `initial position reached; entering work mode`.

Kinematics and simulation use `empty.urdf`; RViz displays `gr100_full.urdf`.

## Installation

### Prerequisites

- **ROS 2 Humble** is installed; use **ROS 1 Noetic** for ROS 1 compatibility.
- Python 3, `pip3`, Git, and the build tools for the selected ROS version are installed.
- A compatible Linux input joystick is connected.

### 1. Install Python Dependencies

```shell
pip3 install \
    'hex-util-msg>=0.1.0' \
    'hex-util-ros>=0.1.0' \
    'hex-driver-robot>=0.1.0' \
    evdev
```

### 2. Create and Enter the Workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 3. Clone ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_joystick.git
git clone https://github.com/hexfellow/hex_ros_teleop_joystick.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 4. Build

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

## Topics

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribe | `manip_state` | `hex_ros_msgs/msg/HexRosRoboManipStateStamped` | Robot arm state message |
| Subscribe | `teleop_joy_state` | `hex_ros_msgs/msg/HexRosTeleopJoystickStateStamped` | Joystick state message |
| Publish | `manip_ctrl` | `hex_ros_msgs/msg/HexRosRoboManipCtrlStamped` | Robot arm control message |
| Publish | `target_marker_array` | `visualization_msgs/msg/MarkerArray` | Target pose marker message |

The target visualization uses the `base_link` frame. The yellow sphere represents the target position, and the red, green, and blue arrows represent the target local X, Y, and Z axes. The markers represent the target state, not measured end-effector feedback. The actual end-effector state is represented by `joint_states` and TF.

## Joystick Mapping

| Input | Function |
|---|---|
| `axis_ly` | Target X translation |
| `axis_lx` | Target Y translation |
| `axis_ry` | Target Z translation |
| `axis_rx` | End-effector yaw about the Z axis |
| `hat_x` | End-effector roll |
| `hat_y` | End-effector pitch |
| `btn_a` | Move the gripper to `gripper_target` while held, then return to zero |
| `btn_b` | Return to `stable_joint` |
| `btn_x` | Exit the node |

## Parameters

Defaults come from `config/<ros_version>/joy_arm.yaml`; launch sets `model_urdf`. Initialization/reset uses JNT; the work phase uses MIT.

| Parameter | Default | Description |
|---|---:|---|
| `control_rate` | `1000.0` | Control-loop frequency used to calculate `dt` |
| `model_urdf` | `""` | URDF path used by joy_arm FK/IK |
| `model_frame_id` | `base_link` | Frame ID for target markers |
| `pose_end_in_flange` | `[0.187, 0, 0, 1, 0, 0, 0]` | End pose relative to the flange |
| `joystick_deadzone` | `0.1` | Deadzone for analog joystick LX/LY/RX/RY |
| `joystick_linear_xy` | `1.0` | Scale from LX/LY to target X/Y instantaneous linear velocity |
| `joystick_linear_z` | `1.0` | Scale from RY to target Z instantaneous linear velocity |
| `joystick_angular` | `1.0` | Scale from RX/hat to target angular velocity |
| `joint_step_limit` | `0.05` | Maximum joint step per cycle during stable/reset |
| `position_step_limit` | `0.01` | Maximum Cartesian step sent to IK per cycle |
| `stable_radius_min` | `0.2` | Minimum distance from the base origin to the stable point |
| `stable_radius_max` | `0.9` | Maximum distance from the base origin to the stable point |
| `stable_joint` | `[0, -0.02, 2.6, -1, 0, 0]` | Initialization and reset joint target |
| `stable_in_end` | `[0, 0, 0.17, 0.707, 0, -0.707, 0]` | Stable-point pose relative to the end frame |
| `gravity` | `[0, 0, -9.81]` | Gravity acceleration vector |
| `arm_stable_kp/kd` | See YAML | Stable/reset arm gains |
| `grip_stable_kp/kd` | See YAML | Stable/reset gripper gains |
| `arm_impedance_kp/kd` | See YAML | Work-phase arm gains |
| `grip_impedance_kp/kd` | See YAML | Work-phase gripper gains |
| `gripper_enabled` | `true` | Enable gripper control target generation |
| `gripper_target` | `0.6` | Gripper target while A is held |

## Project Structure

```text
hex_ros_demo_joystick/
├── config/
│   ├── ros1/
│   │   ├── joy_arm.rviz                     # ROS 1 RViz configuration
│   │   └── joy_arm.yaml                     # ROS 1 node parameters
│   └── ros2/
│       ├── joy_arm.rviz                     # ROS 2 RViz configuration
│       └── joy_arm.yaml                     # ROS 2 node parameters
├── hex_ros_demo_joystick/
│   ├── arm_utils/
│   │   ├── __init__.py                      # arm_utils package initializer
│   │   ├── interface_base.py                # ROS interface base class
│   │   ├── ros1_interface.py                # ROS 1 interface
│   │   └── ros2_interface.py                # ROS 2 interface
│   ├── __init__.py                          # Python package initializer
│   └── joy_arm.py                           # Joystick control node
├── launch/
│   ├── ros1/
│   │   └── sim_joy_arm.launch               # ROS 1 simulation launch file
│   └── ros2/
│       └── sim_joy_arm.launch.py            # ROS 2 simulation launch file
├── resource/
│   └── hex_ros_demo_joystick
├── .gitignore
├── CMakeLists.txt
├── LICENSE
├── package.xml
├── README_cn.md
├── README.md
├── setup.cfg
└── setup.py
```
