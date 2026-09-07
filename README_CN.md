# hex_ros_demo_joystick
**中文** | [English](README.md)

## 目录

- [1. 包简介](#1-包简介)
- [2. 包结构](#2-包结构)
- [3. 话题接口](#3-话题接口)
- [4. 手柄映射](#4-手柄映射)
- [5. 控制模式](#5-控制模式)
- [6. 参数说明](#6-参数说明)
- [7. 依赖关系](#7-依赖关系)
- [8. 快速使用](#8-快速使用)

## 1. 包简介

`hex_ros_demo_joystick` 是基于手柄控制的 ROS demo 包。
- 本阶段仅支持 MuJoCo。
- 包含一个手柄控制机械臂的节点。
## 2. 包结构

```text
hex_ros_demo_joystick/
├── arm_utils/                         # ROS 1/ROS 2 接口实现
│   ├── interface_base.py              # 公共接口和消息队列
│   ├── ros1_interface.py              # ROS 1 rospy 实现
│   └── ros2_interface.py              # ROS 2 rclpy 实现
├── config/
│   ├── ros1/
│   │   ├── joy_arm.yaml               # ROS 1 joy_arm 参数
│   │   └── joy_arm.rviz               # ROS 1 RViz 配置
│   └── ros2/
│       ├── joy_arm.yaml               # ROS 2 joy_arm 参数
│       └── joy_arm.rviz               # ROS 2 RViz 配置
├── launch/
│   ├── ros1/sim_joy_arm.launch        # ROS 1 仿真启动
│   └── ros2/sim_joy_arm.launch.py     # ROS 2 仿真启动
├── hex_ros_demo_joystick/
│   └── joy_arm.py                     # 手柄机械臂控制节点
├── resource/hex_ros_demo_joystick
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README_CN.md
```

## 3. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | 机械臂和夹爪状态 |
| 订阅 | `teleop_joy_state` | `hex_ros_msgs/(msg/)HexRosTeleopJoystickStateStamped` | 手柄状态 |
| 发布 | `manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | MIT/JNT 机械臂和夹爪控制命令 |
| 发布 | `target_marker_array` | `visualization_msgs/(msg/)MarkerArray` | 目标位置球体和目标坐标轴 |

目标可视化使用 `base_link` 坐标系：黄色球体表示目标位置，红色/绿色/蓝色箭头分别表示目标局部 X/Y/Z 轴。
该标记是目标状态，不是实际末端反馈；实际末端由 `joint_states` 和 TF 表示。

## 4. 手柄映射

### arm
| 输入 | 作用 |
|---|---|
| `axis_ly` | 目标 X 平移 |
| `axis_lx` | 目标 Y 平移 |
| `axis_ry` | 目标 Z 平移 |
| `axis_rx` | 末端Yaw，绕 Z 轴旋转 |
| `hat_x` | 末端Roll |
| `hat_y` | 末端Pitch |
| `btn_a` | 按住时夹爪到 `gripper_target`，释放时回到零 |
| `btn_b` | 回到 `stable_joint` |
| `btn_x` | 退出节点 |


## 5. 控制模式

- 机械臂：JNT（初始化和复位），MIT（工作阶段）。
- 夹爪：JNT（初始化和复位），MIT（工作阶段）。

### MIT 模式使用警告

不合适的 `kp`/`kd` 参数可能造成剧烈运动或设备损坏。

> 请在安全区域操作，并确保急停装置可用。

## 6. 参数说明

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `control_rate` | `1000.0` | 控制循环频率，用于计算 `dt` |
| `model_urdf` | `""` | joy_arm FK/IK 使用的 URDF 路径 |
| `pose_end_in_flange` | `[0.187, 0, 0, 1, 0, 0, 0]` | 末端相对 flange 的位姿 |
| `joystick_deadzone` | `0.1` | 模拟摇杆 LX/LY/RX/RY 的死区 |
| `joystick_linear_xy` | `1.0` | LX/LY 映射到目标 X/Y 瞬时线速度的倍率 |
| `joystick_linear_z` | `1.0` | RY 映射到目标 Z 瞬时线速度的倍率 |
| `joystick_angular` | `1.0` | RX/hat 映射到目标角速度的倍率 |
| `joint_step_limit` | `0.05` | stable/reset 每周期最大关节步长 |
| `position_step_limit` | `0.01` | 送入 IK 前每周期最大笛卡尔步长 |
| `stable_radius_min` | `0.2` | stable 点到 base 原点的最小距离 |
| `stable_radius_max` | `0.9` | stable 点到 base 原点的最大距离 |
| `stable_joint` | `[0, -0.02, 2.6, -1, 0, 0]` | 初始化和复位关节目标 |
| `stable_in_end` | `[0, 0, 0.17, 0.707, 0, -0.707, 0]` | stable 点相对末端的位姿 |
| `gravity` | `[0, 0, -9.81]` | 重力加速度向量 |
| `arm_stable_kp/kd` | 见 YAML | stable/reset 阶段机械臂增益 |
| `grip_stable_kp/kd` | 见 YAML | stable/reset 阶段夹爪增益 |
| `arm_impedance_kp/kd` | 见 YAML | 工作阶段机械臂增益 |
| `grip_impedance_kp/kd` | 见 YAML | 工作阶段夹爪增益 |
| `gripper_enabled` | `true` | 是否生成夹爪控制目标 |
| `gripper_target` | `0.6` | A 按住时的夹爪目标位置 |

## 7. 依赖关系

### Python 包

```shell
pip3 install 'hex-util-msg>=0.1.0'
pip3 install 'hex-util-ros>=0.1.0a4'
pip3 install 'hex-driver-robot>=0.1.1'
```

### ROS 包

创建工作空间并获取依赖仓库：

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_joystick.git
git clone https://github.com/hexfellow/hex_ros_teleop_joystick.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

- `hex_ros_msgs`：提供机械臂、夹爪和手柄消息定义。
- `hex_ros_demo_joystick`：提供 `joy_arm` 控制节点。
- `hex_ros_teleop_joystick`：读取系统手柄并发布 `teleop_joy_state`。
- `hex_ros_sim_archer_y6`：提供 Archer Y6 MuJoCo 仿真和 `manip_state`。
- `hex_ros_urdf_archer_y6`：提供 FK/IK 使用的 `empty.urdf` 和 RViz 显示使用的 `gr100_full.urdf`。

此外还需要 ROS 发行版提供的 `robot_state_publisher`、`rviz`/`rviz2` 和 `visualization_msgs`。本包当前仅支持 MuJoCo 仿真，不连接真实机械臂或 ZMQ server。

## 8. 快速使用

### 1. 创建工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. 克隆仓库

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_joystick.git
git clone https://github.com/hexfellow/hex_ros_teleop_joystick.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 3. 编译工作空间

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

### 4. 使用包

本包的 launch 会启动 MuJoCo、手柄读取节点、`joy_arm` 和本包的 RViz 配置。参数文件和 RViz 配置分别为：

```text
config/<ros_version>/joy_arm.yaml
config/<ros_version>/joy_arm.rviz
```

**ROS 2：**

```shell
ros2 launch hex_ros_demo_joystick sim_joy_arm.launch.py \\
    viewer:=true rviz:=true device_path:=/dev/input/eventX
```

**ROS 1：**

```shell
source /home/hexfellow/work/uv.sh
roslaunch hex_ros_demo_joystick sim_joy_arm.launch \\
    viewer:=true rviz:=true device_path:=/dev/input/eventX
```

启动参数：

| 参数 | 说明 |
|---|---|
| `viewer` | 是否启动 MuJoCo viewer。 |
| `rviz` | 是否启动本包 RViz。 |
| `device_path` | 手柄设备路径；留空时自动探测。 |
| `use_sim_time` | 是否使用仿真时间；仿真场景建议设置为 `true`。 |
