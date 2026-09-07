#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import numpy as np
import rospy

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
    """Provide ROS 1 communication for the joystick arm demo."""

    def __init__(self, name: str = "unknown") -> None:
        super(DataInterface, self).__init__(name=name)

        ### ros node
        # Keep the launch-assigned name so private parameters loaded by the
        # ROS 1 launch file (for example ``~control_rate``) resolve to the
        # same node namespace.  Anonymous naming would move the node to a
        # generated name and make those parameters unavailable.
        rospy.init_node(name, anonymous=False)
        self._rate_param["ros"] = rospy.get_param(
            "~control_rate", 1000.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### parameters
        defaults = {
            "joystick_deadzone": 0.1,
            "joystick_linear_xy": 1.0,
            "joystick_linear_z": 1.0,
            "joystick_angular": 1.0,
            "joint_step_limit": 0.05,
            "position_step_limit": 0.01,
            "stable_radius_min": 0.2,
            "stable_radius_max": 0.9,
            "stable_joint": [
                0.0, -0.02, 2.6, -1.0, 0.0, 0.0],
            "stable_in_end": [
                0.0, 0.0, 0.17, 0.7071068, 0.0, -0.7071068, 0.0],
            "gravity": [0.0, 0.0, -9.81],
            "arm_stable_kp": [200.0] * 6,
            "arm_stable_kd": [5.0] * 6,
            "grip_stable_kp": [10.0],
            "grip_stable_kd": [0.5],
            "arm_impedance_kp": [200.0] * 6,
            "arm_impedance_kd": [5.0] * 6,
            "grip_impedance_kp": [10.0],
            "grip_impedance_kd": [0.5],
            "gripper_enabled": True,
            "gripper_target": 0.6,
        }
        self._joy_param = {
            param_name: rospy.get_param("~" + param_name, default)
            for param_name, default in defaults.items()
        }
        self._model_param = {
            "urdf": rospy.get_param('~model_urdf', ""),
            "frame_id": rospy.get_param('~model_frame_id', "base_link"),
            "pose_end_in_flange": list(rospy.get_param(
                '~pose_end_in_flange',
                [0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])), 
        }

        ### publisher
        self.__manip_ctrl_pub = rospy.Publisher(
            'manip_ctrl',
            HexRosRoboManipCtrlStamped,
            queue_size=10,
        )
        self.__target_marker_pub = rospy.Publisher(
            'target_marker_array',
            MarkerArray,
            queue_size=10,
        )

        ### subscriber
        self.__manip_state_sub = rospy.Subscriber(
            'manip_state',
            HexRosRoboManipStateStamped,
            self.__manip_state_callback,
        )
        self.__joy_sub = rospy.Subscriber(
            'teleop_joy_state',
            HexRosTeleopJoystickStateStamped,
            self.__joy_callback,
        )
        self.__manip_state_sub
        self.__joy_sub

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def ok(self) -> bool:
        """Return whether ROS is still running."""
        return not rospy.is_shutdown()

    def shutdown(self) -> None:
        """Release ROS 1 resources."""
        if not rospy.is_shutdown():
            rospy.signal_shutdown("DataInterface shutdown")

    def sleep(self) -> None:
        """Sleep for one control period."""
        self.__rate.sleep()

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    ####################
    ### publishers
    ####################
    def pub_manip_ctrl(self, out: HexDcRoboManipCtrl) -> None:
        msg = HexRosRoboManipCtrlStamped()
        msg.header.stamp = rospy.Time.now()
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
        marker_ns = "joy_arm_target"
        sphere_scale = 0.00003
        arrow_diameter = 0.008
        arrow_length = 0.08
        axis_colors = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )

        position = np.asarray(position, dtype=np.float64).reshape(3)
        w, x, y, z = np.asarray(quaternion, dtype=np.float64)
        rotation = np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ])
        stamp = rospy.Time.now()

        def make_marker(marker_id: int, marker_type: int) -> Marker:
            marker = Marker()
            marker.header.frame_id = self._model_param["frame_id"]
            marker.header.stamp = stamp
            marker.ns = marker_ns
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            return marker

        marker_array = MarkerArray()
        sphere = make_marker(0, Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = sphere_scale
        sphere.color.r = 1.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 0.8
        sphere.pose.position = Point(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
        )
        sphere.pose.orientation.w = 1.0
        marker_array.markers.append(sphere)

        for marker_id, (axis, color) in enumerate(
                zip(rotation.T, axis_colors), start=1):
            arrow = make_marker(marker_id, Marker.ARROW)
            arrow.scale.x = arrow_diameter
            arrow.scale.y = arrow_diameter
            arrow.scale.z = arrow_diameter
            arrow.color.r, arrow.color.g, arrow.color.b = color
            arrow.color.a = 1.0
            tip = position + arrow_length * axis
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
    def __manip_state_callback(
            self, msg: HexRosRoboManipStateStamped) -> None:
        self._manip_state_deque.append(self.__manip_state_msg_to_dc(msg))

    def __joy_callback(
            self, msg: HexRosTeleopJoystickStateStamped) -> None:
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
    def __pose_to_dc(pose) -> HexDcBasePose:
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

    def __manip_state_msg_to_dc(
            self,
            msg: HexRosRoboManipStateStamped) -> HexDcRoboManipStateStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.secs),
                nsecs=int(msg.header.stamp.nsecs),
            ),
            frame_id=msg.header.frame_id,
        )

        arm_msg = msg.manip_state.arm_state
        arm_state = HexDcRoboArmState(
            jnt=self.__jnt_state_to_dc(arm_msg.jnt),
            pose=self.__pose_to_dc(arm_msg.pose),
        )

        grip_msg = msg.manip_state.grip_state
        grip_state = HexDcRoboGripState(jnt=self.__jnt_state_to_dc(
            grip_msg.jnt), )

        return HexDcRoboManipStateStamped(
            header=header,
            manip_state=HexDcRoboManipState(
                arm_state=arm_state,
                grip_state=grip_state,
            ),
        )
