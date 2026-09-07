#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import threading

import numpy as np
import rclpy
import rclpy.node
from rclpy.logging import LoggingSeverity

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboArmCtrl,
    HexRosRoboGripCtrl,
    HexRosRoboManipCtrl,
    HexRosRoboManipCtrlStamped,
    HexRosRoboManipStateStamped,
    HexRosTeleopJoystickStateStamped,
)

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntState,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmState,
    HexDcRoboGripCtrl,
    HexDcRoboGripState,
    HexDcRoboManipCtrl,
    HexDcRoboManipState,
    HexDcRoboManipStateStamped,
)
from hex_util_msg.dataclass.dataclass_teleop import HexDcTeleopJoystickState

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__logger.set_level(LoggingSeverity.DEBUG)
        self.__node.declare_parameter('control_rate', 1000.0)
        self._rate_param["ros"] = self.__node.get_parameter('control_rate').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### parameters
        self.__node.declare_parameter('model_urdf', "")
        self.__node.declare_parameter('model_frame_id', "base_link")
        self.__node.declare_parameter(
            'pose_end_in_flange',
            [0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        )
        # Joystick and target-motion parameters.
        self.__node.declare_parameter("joystick_deadzone", 0.1)
        self.__node.declare_parameter("joystick_linear_xy", 1.0)
        self.__node.declare_parameter("joystick_linear_z", 1.0)
        self.__node.declare_parameter("joystick_angular", 1.0)
        self.__node.declare_parameter("joint_step_limit", 0.05)
        self.__node.declare_parameter("position_step_limit", 0.01)
        self.__node.declare_parameter("stable_radius_min", 0.2)
        self.__node.declare_parameter("stable_radius_max", 0.9)
        self.__node.declare_parameter(
            "stable_joint",
            [0.0, -0.02, 2.6, -1.0, 0.0, 0.0],
        )
        self.__node.declare_parameter(
            "stable_in_end",
            [0.0, 0.0, 0.17, 0.7071068, 0.0, -0.7071068, 0.0],
        )

        # Controller and gripper parameters.
        self.__node.declare_parameter("gravity", [0.0, 0.0, -9.81])
        self.__node.declare_parameter("arm_stable_kp", [200.0] * 6)
        self.__node.declare_parameter("arm_stable_kd", [5.0] * 6)
        self.__node.declare_parameter("grip_stable_kp", [10.0])
        self.__node.declare_parameter("grip_stable_kd", [0.5])
        self.__node.declare_parameter("arm_impedance_kp", [200.0] * 6)
        self.__node.declare_parameter("arm_impedance_kd", [5.0] * 6)
        self.__node.declare_parameter("grip_impedance_kp", [10.0])
        self.__node.declare_parameter("grip_impedance_kd", [0.5])
        self.__node.declare_parameter("gripper_enabled", True)
        self.__node.declare_parameter("gripper_target", 0.6)

        joy_parameter_names = (
            "joystick_deadzone", "joystick_linear_xy", "joystick_linear_z",
            "joystick_angular", "joint_step_limit",
            "position_step_limit", "stable_radius_min",
            "stable_radius_max", "stable_joint", "stable_in_end",
            "gravity", "arm_stable_kp", "arm_stable_kd", "grip_stable_kp", "grip_stable_kd",
            "arm_impedance_kp", "arm_impedance_kd", "grip_impedance_kp",
            "grip_impedance_kd", "gripper_enabled", "gripper_target",
        )
        self._joy_param = {
            parameter_name: self.__node.get_parameter(parameter_name).value
            for parameter_name in joy_parameter_names
        }
        self._model_param = {
            "urdf": self.__node.get_parameter('model_urdf').value,
            "frame_id": self.__node.get_parameter('model_frame_id').value,
            "pose_end_in_flange": list(self.__node.get_parameter(
                'pose_end_in_flange').value),
        }

        ### publisher
        self.__manip_ctrl_pub = self.__node.create_publisher(
            HexRosRoboManipCtrlStamped,
            'manip_ctrl',
            10,
        )
        self.__target_marker_pub = self.__node.create_publisher(
            MarkerArray,
            'target_marker_array',
            10,
        )

        ### subscriber
        self.__manip_state_sub = self.__node.create_subscription(
            HexRosRoboManipStateStamped,
            'manip_state',
            self.__manip_state_callback,
            10,
        )
        self.__joy_sub = self.__node.create_subscription(
            HexRosTeleopJoystickStateStamped,
            'teleop_joy_state',
            self.__joy_callback,
            10,
        )
        self.__manip_state_sub
        self.__joy_sub

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    ####################
    ### publishers
    ####################
    def pub_manip_ctrl(self, out: HexDcRoboManipCtrl):
        msg = HexRosRoboManipCtrlStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.manip_ctrl = HexRosRoboManipCtrl(
            arm_ctrl=self.__arm_ctrl_to_msg(out.arm_ctrl),
            grip_ctrl=self.__grip_ctrl_to_msg(out.grip_ctrl),
        )
        self.__manip_ctrl_pub.publish(msg)

    def pub_target_marker_array(
        self,
        position: np.ndarray,
        quaternion: np.ndarray) -> None:
        """Publish target position and orientation axes for RViz."""

        MARKER_NS = "joy_arm_target"
        SPHERE_SCALE = 0.00003
        ARROW_DIAMETER = 0.008
        ARROW_LENGTH = 0.08
        AXIS_COLORS = ((1.0, 0.0, 0.0),   # x: red
                    (0.0, 1.0, 0.0),   # y: green
                    (0.0, 0.0, 1.0))   # z: blue

        position = np.asarray(position, dtype=np.float64).reshape(3)
        w, x, y, z = np.asarray(quaternion, dtype=np.float64)

        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

        stamp = self.__node.get_clock().now().to_msg()

        def make_marker(marker_id: int, marker_type: int) -> Marker:
            marker = Marker()
            marker.header.frame_id = self._model_param["frame_id"]
            marker.header.stamp = stamp
            marker.ns = MARKER_NS
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            return marker

        marker_array = MarkerArray()

        sphere = make_marker(0, Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = SPHERE_SCALE
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 1.0, 0.0, 0.8
        sphere.pose.position = Point(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
        )
        sphere.pose.orientation.w = 1.0
        marker_array.markers.append(sphere)

        for marker_id, (axis, color) in enumerate(zip(rotation.T, AXIS_COLORS), start=1):
            arrow = make_marker(marker_id, Marker.ARROW)
            arrow.scale.x = arrow.scale.y = arrow.scale.z = ARROW_DIAMETER
            arrow.color.r, arrow.color.g, arrow.color.b = color
            arrow.color.a = 1.0
            tip = position + ARROW_LENGTH * axis
            arrow.points = [
                Point(
                    x=float(position[0]),
                    y=float(position[1]),
                    z=float(position[2]),
                ),
                Point(
                    x=float(tip[0]),
                    y=float(tip[1]),
                    z=float(tip[2]),
                ),
            ]
            marker_array.markers.append(arrow)

        self.__target_marker_pub.publish(marker_array)
            
    @staticmethod
    def __jnt_to_msg(jnt) -> HexRosJnt:
        return HexRosJnt(
            pos=np.asarray(jnt.pos, dtype=np.float64).tolist(),
            vel=np.asarray(jnt.vel, dtype=np.float64).tolist(),
            eff=np.asarray(jnt.eff, dtype=np.float64).tolist(),
            kp=np.asarray(jnt.kp, dtype=np.float64).tolist(),
            kd=np.asarray(jnt.kd, dtype=np.float64).tolist(),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64).tolist(),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64).tolist(),
        )

    @staticmethod
    def __arm_ctrl_to_msg(arm: HexDcRoboArmCtrl) -> HexRosRoboArmCtrl:
        return HexRosRoboArmCtrl(
            ctrl_mode=int(arm.ctrl_mode),
            grav=Vector3(x=arm.grav.x, y=arm.grav.y, z=arm.grav.z),
            jnt=DataInterface.__jnt_to_msg(arm.jnt),
            pose=Pose(
                position=Point(
                    x=arm.pose.position.x,
                    y=arm.pose.position.y,
                    z=arm.pose.position.z,
                ),
                orientation=Quaternion(
                    x=arm.pose.orientation.x,
                    y=arm.pose.orientation.y,
                    z=arm.pose.orientation.z,
                    w=arm.pose.orientation.w,
                ),
            ),
        )

    @staticmethod
    def __grip_ctrl_to_msg(grip: HexDcRoboGripCtrl) -> HexRosRoboGripCtrl:
        return HexRosRoboGripCtrl(
            ctrl_mode=int(grip.ctrl_mode),
            jnt=DataInterface.__jnt_to_msg(grip.jnt),
        )

    ####################
    ### subscribers
    ####################
    def __manip_state_callback(self, msg: HexRosRoboManipStateStamped):
        self._manip_state_deque.append(self.__manip_state_msg_to_dc(msg))

    def __joy_callback(self, msg: HexRosTeleopJoystickStateStamped):
        self._joy_deque.append(self.__joy_msg_to_dc(msg))

    @staticmethod
    def __joy_msg_to_dc(
            msg: HexRosTeleopJoystickStateStamped) -> HexDcTeleopJoystickState:
        joy = msg.joystick_state
        fields = (
            "btn_x", "btn_y", "btn_a", "btn_b", "btn_lb", "btn_rb",
            "btn_lt", "btn_rt", "btn_lthumb", "btn_rthumb", "axis_lx",
            "axis_ly", "axis_rx", "axis_ry", "axis_lt", "axis_rt",
            "hat_x", "hat_y",
        )
        kwargs = {
            field: bool(getattr(joy, field)) if field.startswith("btn_")
            else float(getattr(joy, field))
            for field in fields
        }
        return HexDcTeleopJoystickState(**kwargs)

    @staticmethod
    def __jnt_state_to_dc(jnt) -> HexDcBaseJntState:
        return HexDcBaseJntState(
            position=np.asarray(jnt.position, dtype=np.float64),
            velocity=np.asarray(jnt.velocity, dtype=np.float64),
            effort=np.asarray(jnt.effort, dtype=np.float64),
        )

    @staticmethod
    def __pose_to_dc(pose: Pose) -> HexDcBasePose:
        return HexDcBasePose(
            position=HexDcBaseVector3(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
            ),
            orientation=HexDcBaseQuaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
        )

    @staticmethod
    def __manip_state_msg_to_dc(
            msg: HexRosRoboManipStateStamped) -> HexDcRoboManipStateStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.sec),
                nsecs=int(msg.header.stamp.nanosec),
            ),
            frame_id=msg.header.frame_id,
        )

        arm_msg = msg.manip_state.arm_state
        arm_state = HexDcRoboArmState(
            jnt=DataInterface.__jnt_state_to_dc(arm_msg.jnt),
            pose=DataInterface.__pose_to_dc(arm_msg.pose),
        )

        grip_msg = msg.manip_state.grip_state
        grip_state = HexDcRoboGripState(
            jnt=DataInterface.__jnt_state_to_dc(grip_msg.jnt), )

        return HexDcRoboManipStateStamped(
            header=header,
            manip_state=HexDcRoboManipState(
                arm_state=arm_state,
                grip_state=grip_state,
            ),
        )
