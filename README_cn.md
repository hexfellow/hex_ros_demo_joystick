# hex_ros_demo_joystick — 手柄控制机械臂演示

**中文** | [English](README.md)

## 目录

- [项目概述](#项目概述)
- [快速使用](#快速使用)
- [安装](#安装)
- [话题接口](#话题接口)
- [手柄映射](#手柄映射)
- [参数说明](#参数说明)
- [项目结构](#项目结构)

## 项目概述

`hex_ros_demo_joystick` 是 Archer Y6 的手柄遥操作演示包。它读取 Linux 手柄输入，将末端平移、旋转和夹爪操作转换为机械臂控制指令，并在 MuJoCo 中执行和可视化目标。

主要提供：

- 完整的 MuJoCo、手柄节点、控制节点和 RViz 启动入口；
- 笛卡尔位置与姿态控制；
- 夹爪、复位和退出按键映射。

当前提供 MuJoCo 仿真入口。本包支持 **ROS 2 Humble**，兼容 **ROS 1 Noetic**。

## 快速使用

> 请先完成[安装](#安装)，再选择以下启动入口。

### 1. 启动仿真

`sim_joy_arm` 会启动 MuJoCo、手柄读取节点、`joy_arm` 和本包的 RViz 配置。参数文件和 RViz 配置分别为：

```text
config/<ros_version>/joy_arm.yaml
config/<ros_version>/joy_arm.rviz
```

手柄读取节点从 `hex_ros_teleop_joystick` 的参数文件读取 `device_path`。默认值为空，会自动检测第一个手柄，因此建议先直接启动：

**ROS 2：**

```shell
ros2 launch hex_ros_demo_joystick sim_joy_arm.launch.py viewer:=true rviz:=true
```

**ROS 1：**

```shell
roslaunch hex_ros_demo_joystick sim_joy_arm.launch viewer:=true rviz:=true
```

如果自动检测无效或存在多个输入设备，先查询稳定的设备路径：

```shell
ls -l /dev/input/by-id/
```

然后在工作空间源码中的对应下游参数文件里设置 `device_path`：

```text
# ROS 2
<your_ws>/src/hex_ros_teleop_joystick/config/ros2/params.yaml

# ROS 1
<your_ws>/src/hex_ros_teleop_joystick/config/ros1/params.yaml
```

例如：

```yaml
device_path: "/dev/input/by-id/<joystick-device>"
```

ROS 2 中请保持该参数位于 `/**/teleop_joystick.ros__parameters` 下。修改后重新构建并 source 工作空间。

也可使用 `/dev/input/eventX`，其中 `X` 可通过 `ls -l /dev/input/` 或上述 `by-id` 软链接目标确认。路径错误会导致手柄读取节点无法正常运行。

如果设备路径存在但仍无法打开，请检查当前用户对该设备的读取权限和所属用户组；不要通过宽泛权限绕过设备访问控制。若终端显示 `no joystick device found`，launch 可能仍保持运行，但不会发布有效手柄状态；请先修复设备路径或权限，并用 `ros2 topic echo /teleop_joy_state`（ROS 1 使用 `rostopic echo /teleop_joy_state`）确认输入。

启动参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `viewer` | `true` | 是否启动 MuJoCo viewer。 |
| `rviz` | `true` | 是否启动本包 RViz。 |
| `use_sim_time` | `true` | 是否使用仿真时间。 |

启动后，仿真机械臂会先自动运动到 `stable_joint`；终端显示 `initial position reached; entering work mode` 后进入正常手柄控制阶段。

运动学与仿真使用 `empty.urdf`；RViz 使用 `gr100_full.urdf` 显示模型。

## 安装

### 前置条件

- 已安装 **ROS 2 Humble**；使用 ROS 1 时安装 **ROS 1 Noetic**。
- 已安装 Python 3、`pip3`、Git，以及所选 ROS 版本的构建工具。
- 已连接兼容的 Linux 输入设备手柄。

### 1. 安装 Python 依赖

```shell
pip3 install \
    'hex-util-msg>=0.1.0' \
    'hex-util-ros>=0.1.0' \
    'hex-driver-robot>=0.1.0' \
    evdev
```

### 2. 创建并进入工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 3. 克隆 ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_joystick.git
git clone https://github.com/hexfellow/hex_ros_teleop_joystick.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 4. 编译包

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `manip_state` | `hex_ros_msgs/msg/HexRosRoboManipStateStamped` | 机械臂状态消息 |
| 订阅 | `teleop_joy_state` | `hex_ros_msgs/msg/HexRosTeleopJoystickStateStamped` | 手柄状态消息 |
| 发布 | `manip_ctrl` | `hex_ros_msgs/msg/HexRosRoboManipCtrlStamped` | 机械臂控制消息 |
| 发布 | `target_marker_array` | `visualization_msgs/msg/MarkerArray` | 目标位姿标记消息 |

目标可视化使用 `base_link` 坐标系：黄色球体表示目标位置，红色/绿色/蓝色箭头分别表示目标局部 X/Y/Z 轴。
该标记是目标状态，不是实际末端反馈；实际末端由 `joint_states` 和 TF 表示。

## 手柄映射

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

## 参数说明

默认值来自 `config/<ros_version>/joy_arm.yaml`；`model_urdf` 由 launch 设置。初始化/复位使用 JNT，工作阶段使用 MIT。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `control_rate` | `1000.0` | 控制循环频率，用于计算 `dt` |
| `model_urdf` | `""` | joy_arm FK/IK 使用的 URDF 路径 |
| `model_frame_id` | `base_link` | 目标标记的坐标系 |
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

## 项目结构

```text
hex_ros_demo_joystick/
├── config/
│   ├── ros1/
│   │   ├── joy_arm.rviz                     # ROS 1 RViz 配置
│   │   └── joy_arm.yaml                     # ROS 1 节点参数
│   └── ros2/
│       ├── joy_arm.rviz                     # ROS 2 RViz 配置
│       └── joy_arm.yaml                     # ROS 2 节点参数
├── hex_ros_demo_joystick/
│   ├── arm_utils/
│   │   ├── __init__.py                      # arm_utils 包初始化文件
│   │   ├── interface_base.py                # ROS 接口基类
│   │   ├── ros1_interface.py                # ROS 1 接口
│   │   └── ros2_interface.py                # ROS 2 接口
│   ├── __init__.py                          # Python 包初始化文件
│   └── joy_arm.py                           # 手柄控制节点
├── launch/
│   ├── ros1/
│   │   └── sim_joy_arm.launch               # ROS 1 仿真启动文件
│   └── ros2/
│       └── sim_joy_arm.launch.py            # ROS 2 仿真启动文件
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
