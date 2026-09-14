# hex_ros_demo_joystick
[中文](README_cn.md) | **English**

## Table of Contents

- [1. About](#1-about)
- [2. Package Structure](#2-package-structure)
- [3. Topics](#3-topics)
- [4. Joystick Mapping](#4-joystick-mapping)
- [5. Control Modes](#5-control-modes)
- [6. Parameters](#6-parameters)
- [7. Dependencies](#7-dependencies)
- [8. Quick Start](#8-quick-start)

## 1. About

`hex_ros_demo_joystick` is a ROS demo package for controlling an Archer Y6 arm with a joystick.

- This package currently supports MuJoCo only.
- It provides a joystick-controlled arm node.
- It currently provides only the `sim_joy_arm` simulation launch; no real-arm launch is provided.

## 2. Package Structure

```text
hex_ros_demo_joystick/
├── arm_utils/                         # ROS 1 / ROS 2 interface implementations
│   ├── interface_base.py              # Common interface and message queues
│   ├── ros1_interface.py              # ROS 1 rospy implementation
│   └── ros2_interface.py              # ROS 2 rclpy implementation
├── config/
│   ├── ros1/
│   │   ├── joy_arm.yaml               # ROS 1 joy_arm parameters
│   │   └── joy_arm.rviz               # ROS 1 RViz configuration
│   └── ros2/
│       ├── joy_arm.yaml               # ROS 2 joy_arm parameters
│       └── joy_arm.rviz               # ROS 2 RViz configuration
├── launch/
│   ├── ros1/sim_joy_arm.launch        # ROS 1 simulation launch
│   └── ros2/sim_joy_arm.launch.py     # ROS 2 simulation launch
├── hex_ros_demo_joystick/
│   └── joy_arm.py                     # Joystick arm control node
├── resource/hex_ros_demo_joystick
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README_CN.md
```

## 3. Topics

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribe | `manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Arm and gripper state |
| Subscribe | `teleop_joy_state` | `hex_ros_msgs/(msg/)HexRosTeleopJoystickStateStamped` | Joystick state |
| Publish | `manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | Arm and gripper MIT/JNT control command |
| Publish | `target_marker_array` | `visualization_msgs/(msg/)MarkerArray` | Target position sphere and target axes |

The target visualization uses the `base_link` frame. The yellow sphere represents the target position, and the red, green, and blue arrows represent the target local X, Y, and Z axes. The markers represent the target state, not measured end-effector feedback. The actual end-effector state is represented by `joint_states` and TF.

## 4. Joystick Mapping

### Arm

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


## 5. Control Modes

- Arm: JNT during initialization and reset; MIT during the work phase.
- Gripper: JNT during initialization and reset; MIT during the work phase.

### MIT Mode Warning

Incorrect `kp`/`kd` values may cause violent motion or equipment damage.

> Operate in a safe area with emergency stop accessible.

## 6. Parameters

| Parameter | Default | Description |
|---|---:|---|
| `control_rate` | `1000.0` | Control-loop frequency used to calculate `dt` |
| `model_urdf` | `""` | URDF path used by joy_arm FK/IK |
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

## 7. Dependencies

### Python Packages

```shell
pip3 install 'hex-util-msg>=0.1.0'
pip3 install 'hex-util-ros>=0.1.0a4'
pip3 install 'hex-driver-robot>=0.1.1'
```

### ROS Packages

Create a workspace and clone the required repositories:

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src

git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_joystick.git
git clone https://github.com/hexfellow/hex_ros_teleop_joystick.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

- `hex_ros_msgs` provides the arm, gripper, and joystick message definitions.
- `hex_ros_demo_joystick` provides the `joy_arm` control node.
- `hex_ros_teleop_joystick` reads the system joystick and publishes `teleop_joy_state`.
- `hex_ros_sim_archer_y6` provides the Archer Y6 MuJoCo simulation and `manip_state`.
- `hex_ros_urdf_archer_y6` provides `empty.urdf` for FK/IK and `gr100_full.urdf` for RViz display.

The ROS distribution must also provide `robot_state_publisher`, `rviz` or `rviz2`, and `visualization_msgs`. This package currently supports MuJoCo simulation only and does not connect to a real robot or a ZMQ server.

## 8. Quick Start

### 1. Create Workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. Clone Repositories

Run the five `git clone` commands from Section 6. All repositories must be placed under the same workspace `src/` directory.

### 3. Build Workspace

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

### 4. Use the Package

The launch file starts MuJoCo, joystick input, `joy_arm`, and the RViz configuration owned by this package. The package currently supports `sim_joy_arm` only and does not connect to a real arm. The parameter and RViz files are:

```text
config/<ros_version>/joy_arm.yaml
config/<ros_version>/joy_arm.rviz
```

**ROS 2:**

```shell
ros2 launch hex_ros_demo_joystick sim_joy_arm.launch.py \
    viewer:=true rviz:=true device_path:=/dev/input/eventX
```

**ROS 1:**

```shell
source /home/hexfellow/work/uv.sh
roslaunch hex_ros_demo_joystick sim_joy_arm.launch \
    viewer:=true rviz:=true device_path:=/dev/input/eventX
```

Launch arguments:

| Argument | Description |
|---|---|
| `viewer` | Start the MuJoCo viewer. |
| `rviz` | Start the RViz instance owned by this package. |
| `device_path` | Joystick device path; leave empty for automatic detection. |
| `use_sim_time` | Use simulation time; set this to `true` for simulation. |

Current model configuration:

```text
joy_arm FK/IK model: empty.urdf
MuJoCo model URDF: empty.urdf
RViz visual model: gr100_full.urdf
```

`empty.urdf` must be used by both `joy_arm` FK/IK and the MuJoCo state calculation so that they use the same kinematic model. `gr100_full.urdf` is used only for the RViz RobotModel display.

Leave `device_path` empty to auto-detect the first joystick. This package currently supports MuJoCo simulation only and does not provide a real-arm launch.
