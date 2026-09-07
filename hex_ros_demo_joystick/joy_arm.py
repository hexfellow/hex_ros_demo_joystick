#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""Joystick-controlled MuJoCo arm demonstration.

Copyright 2026 Dong Zhaorui. All rights reserved.
Author: Dong Zhaorui 847235539@qq.com
Date: 2026-08-13

The node first moves the arm to a parameter-defined joint position. During
work mode it integrates joystick input into a target pose, constrains that
pose to the configured workspace, solves analytic IK, and publishes MIT joint
commands.
"""

import os
import sys
import threading
import traceback
from typing import Any, Dict, Tuple

import numpy as np
from hex_util_ros import (
    HexDynUtilY6,
    part2trans,
    quat_mul,
    trans2part,
    trans_inv,
)

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)
from arm_utils import DataInterface

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseJntFull,
    HexDcBasePose,
    HexDcBaseQuaternion,
    HexDcBaseVector3,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode,
    HexDcRoboManipCtrl,
)

ARM_DOF = 6
GRIP_DOF = 1


class JoyArm:
    """Control an Archer Y6 MuJoCo arm from a joystick topic."""

    def __init__(self) -> None:
        # Utility.
        self.__data_interface = DataInterface("joy_arm")

        # Parameters.
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__model_param = self.__data_interface.get_model_param()
        self.__param = self.__data_interface.get_joy_param()
        self.__data_interface.logi(
            f"[joy_arm] work rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"[joy_arm] model urdf: {self.__model_param['urdf']}")

        # Dynamics.
        self.__gravity = np.asarray(
            self.__param["gravity"], dtype=np.float64)
        self.__dyn_util = HexDynUtilY6(
            model_path=self.__model_param["urdf"],
            last_link="link_6",
            pose_end_in_flange=np.asarray(
                self.__model_param["pose_end_in_flange"], dtype=np.float64),
            gravity=self.__gravity,
        )

        # Target-motion parameters.
        self.__init_joint = np.asarray(
            self.__param["stable_joint"], dtype=np.float64)
        self.__deadzone_threshold = float(self.__param["joystick_deadzone"])
        self.__linear_speed_xy = float(self.__param["joystick_linear_xy"])
        self.__linear_speed_z = float(self.__param["joystick_linear_z"])
        self.__angular_speed = float(self.__param["joystick_angular"])
        self.__joint_error_limit = float(self.__param["joint_step_limit"])
        self.__position_error_threshold = float(
            self.__param["position_step_limit"])
        self.__stable_distance_min = float(
            self.__param["stable_radius_min"])
        self.__stable_distance_max = float(
            self.__param["stable_radius_max"])
        self.__use_gripper = bool(self.__param["gripper_enabled"])
        self.__grip_max_pos = float(self.__param["gripper_target"])

        # Control presets.
        self.__arm_stable_kp = np.asarray(
            self.__param["arm_stable_kp"], dtype=np.float64)
        self.__arm_stable_kd = np.asarray(
            self.__param["arm_stable_kd"], dtype=np.float64)
        self.__grip_stable_kp = np.asarray(
            self.__param["grip_stable_kp"], dtype=np.float64)
        self.__grip_stable_kd = np.asarray(
            self.__param["grip_stable_kd"], dtype=np.float64)
        self.__arm_impedance_kp = np.asarray(
            self.__param["arm_impedance_kp"], dtype=np.float64)
        self.__arm_impedance_kd = np.asarray(
            self.__param["arm_impedance_kd"], dtype=np.float64)
        self.__grip_impedance_kp = np.asarray(
            self.__param["grip_impedance_kp"], dtype=np.float64)
        self.__grip_impedance_kd = np.asarray(
            self.__param["grip_impedance_kd"], dtype=np.float64)

        # Workspace constraint transforms.
        stable_pose = np.asarray(
            self.__param["stable_in_end"], dtype=np.float64)
        self.__trans_stable_in_end = part2trans(
            stable_pose[:3], stable_pose[3:])
        self.__trans_end_in_stable = trans_inv(
            self.__trans_stable_in_end)

        # Shared target state.
        initial_pose = self.__dyn_util.forward_kinematics(
            self.__init_joint)[-1]
        self._target_lock = threading.Lock()
        self._target: Dict[str, Any] = {
            "tar_pose": (
                np.asarray(initial_pose[0], dtype=np.float64).copy(),
                np.asarray(initial_pose[1], dtype=np.float64).copy(),
            ),
            "tar_vel": np.zeros(3, dtype=np.float64),
            "tar_omega": np.zeros(3, dtype=np.float64),
            "reset_flag": False,
            "quit_flag": False,
            "grip_flag": False,
        }
        self.__previous_buttons = {
            "a": False,
            "b": False,
            "x": False,
        }
        self.__last_arm_command = self.__init_joint.copy()
        self.__reset_active = False

        # Lifecycle.
        self.__stop_event = threading.Event()
        self.__work_dt = 1.0 / max(float(self.__rate_param["ros"]), 1.0)

    def __is_running(self) -> bool:
        """Return whether ROS and the controller are still running."""
        return self.__data_interface.ok() and not self.__stop_event.is_set()

    ##############################################################
    # Lifecycle
    ##############################################################
    def start(self) -> None:
        """Clear the stop state and move the arm to its stable position."""
        self.__stop_event.clear()
        self.__init_process()

    def run(self) -> None:
        """Run joystick control until shutdown or a quit request."""
        try:
            self.__work_process()
        except KeyboardInterrupt:
            pass
        except Exception:
            traceback.print_exc()
        finally:
            self.stop()

    def stop(self) -> None:
        """Stop control and release ROS resources."""
        self.__stop_event.set()
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass

    ##############################################################
    # Control builders
    ##############################################################
    @staticmethod
    def __default_pose() -> HexDcBasePose:
        """Return an identity pose for unused pose-control fields."""
        return HexDcBasePose(
            position=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
            orientation=HexDcBaseQuaternion(
                x=0.0, y=0.0, z=0.0, w=1.0),
        )

    def __build_stable_ctrl(
            self,
            arm_jnt_pos: np.ndarray,
            grip_jnt_pos: np.ndarray) -> HexDcRoboManipCtrl:
        """Build a JNT command for initialization or reset.

        Args:
            arm_jnt_pos: Target arm joint position.
            grip_jnt_pos: Target gripper joint position.

        Returns:
            A stable-position manipulation command.
        """
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.JNT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=arm_jnt_pos.copy(),
                vel=np.zeros(ARM_DOF),
                eff=np.zeros(ARM_DOF),
                kp=self.__arm_stable_kp.copy(),
                kd=self.__arm_stable_kd.copy(),
                lim_vel=15.0 * np.ones(ARM_DOF, dtype=np.float64),
                lim_acc=15.0 * np.ones(ARM_DOF, dtype=np.float64),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.JNT,
            jnt=HexDcBaseJntFull(
                pos=grip_jnt_pos.copy(),
                vel=np.zeros(GRIP_DOF),
                eff=np.zeros(GRIP_DOF),
                kp=self.__grip_stable_kp.copy(),
                kd=self.__grip_stable_kd.copy(),
                lim_vel=np.array([0.5], dtype=np.float64),
                lim_acc=np.array([1.0], dtype=np.float64),
            ),
        )
        return HexDcRoboManipCtrl(
            arm_ctrl=arm_ctrl,
            grip_ctrl=grip_ctrl,
        )

    def __build_impedance_ctrl(
            self,
            arm_jnt_pos: np.ndarray,
            grip_jnt_pos: np.ndarray) -> HexDcRoboManipCtrl:
        """Build an MIT command for joystick impedance control.

        Args:
            arm_jnt_pos: Target arm joint position.
            grip_jnt_pos: Target gripper joint position.

        Returns:
            An impedance manipulation command.
        """
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.MIT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=arm_jnt_pos.copy(),
                vel=np.zeros(ARM_DOF),
                eff=np.zeros(ARM_DOF),
                kp=self.__arm_impedance_kp.copy(),
                kd=self.__arm_impedance_kd.copy(),
                lim_vel=np.zeros(ARM_DOF),
                lim_acc=np.zeros(ARM_DOF),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.MIT,
            jnt=HexDcBaseJntFull(
                pos=grip_jnt_pos.copy(),
                vel=np.zeros(GRIP_DOF),
                eff=np.zeros(GRIP_DOF),
                kp=self.__grip_impedance_kp.copy(),
                kd=self.__grip_impedance_kd.copy(),
                lim_vel=np.zeros(GRIP_DOF),
                lim_acc=np.zeros(GRIP_DOF),
            ),
        )
        return HexDcRoboManipCtrl(
            arm_ctrl=arm_ctrl,
            grip_ctrl=grip_ctrl,
        )

    ##############################################################
    # Target helpers
    ##############################################################
    @staticmethod
    def __deadzone(value: float, threshold: float) -> float:
        """Apply a symmetric joystick_deadzone to one joystick value.

        Args:
            value: Normalized joystick value.
            threshold: Deadzone width.

        Returns:
            Deadzone-compensated joystick value.
        """
        if abs(value) < threshold:
            return 0.0
        return value - np.sign(value) * threshold

    def __calculate_target_motion(
            self, joy_state: Any) -> Tuple[np.ndarray, np.ndarray]:
        """Convert joystick input into target linear and angular velocity.

        Args:
            joy_state: Latest normalized joystick state.

        Returns:
            Target linear velocity and angular velocity.
        """
        joystick_deadzone = self.__deadzone_threshold
        tar_vel = np.array([
            -self.__linear_speed_xy * self.__deadzone(
                joy_state.axis_ly, joystick_deadzone),
            -self.__linear_speed_xy * self.__deadzone(
                joy_state.axis_lx, joystick_deadzone),
            -self.__linear_speed_z * self.__deadzone(
                joy_state.axis_ry, joystick_deadzone),
        ], dtype=np.float64)
        tar_omega = self.__angular_speed * np.array([
            joy_state.hat_x,
            -joy_state.hat_y,
            -self.__deadzone(joy_state.axis_rx, joystick_deadzone),
        ], dtype=np.float64)
        return tar_vel, tar_omega

    @staticmethod
    def __interpolate_joint(
            jnt_pos: np.ndarray,
            tar_jnt_pos: np.ndarray,
            error_limit: float) -> Tuple[np.ndarray, bool]:
        """Limit the maximum joint displacement in one control cycle.

        Args:
            jnt_pos: Current joint position.
            tar_jnt_pos: Final target joint position.
            error_limit: Maximum displacement of any joint.

        Returns:
            The interpolated joint target and whether interpolation continues.
        """
        jnt_err = tar_jnt_pos - jnt_pos
        max_err = np.max(np.abs(jnt_err))
        if max_err <= error_limit:
            return tar_jnt_pos.copy(), False
        return jnt_pos + jnt_err / max_err * error_limit, True

    @staticmethod
    def __interpolate_position(
            pos: np.ndarray,
            tar_pos: np.ndarray,
            error_limit: float) -> np.ndarray:
        """Limit Cartesian displacement using its Euclidean norm.

        Args:
            pos: Current Cartesian position.
            tar_pos: Final target Cartesian position.
            error_limit: Maximum Cartesian displacement.

        Returns:
            The interpolated Cartesian target position.
        """
        pos_err = tar_pos - pos
        error_norm = np.linalg.norm(pos_err)
        if error_norm <= error_limit:
            return tar_pos.copy()
        return pos + pos_err / error_norm * error_limit

    def __copy_target(self) -> Dict[str, Any]:
        """Return a consistent copy of the shared target state."""
        with self._target_lock:
            tar_pose = self._target["tar_pose"]
            return {
                "tar_pose": (tar_pose[0].copy(), tar_pose[1].copy()),
                "tar_vel": self._target["tar_vel"].copy(),
                "tar_omega": self._target["tar_omega"].copy(),
                "reset_flag": self._target["reset_flag"],
                "quit_flag": self._target["quit_flag"],
                "grip_flag": self._target["grip_flag"],
            }

    def __update_target(self, joy_state: Any) -> None:
        """Update all joystick-derived target values atomically.

        Args:
            joy_state: Latest normalized joystick state.
        """
        tar_vel, tar_omega = self.__calculate_target_motion(joy_state)
        with self._target_lock:
            pos, quat = self._target["tar_pose"]
            tar_pos = pos + tar_vel * self.__work_dt
            delta_angle = tar_omega * self.__work_dt
            angle_norm = np.linalg.norm(delta_angle)
            tar_quat = quat.copy()
            if angle_norm > 1e-9:
                axis = delta_angle / angle_norm
                delta_quat = np.array([
                    np.cos(angle_norm / 2.0),
                    *(axis * np.sin(angle_norm / 2.0)),
                ])
                tar_quat = quat_mul(tar_quat, delta_quat)

            self._target["tar_pose"] = (tar_pos, tar_quat)
            self._target["tar_vel"] = tar_vel
            self._target["tar_omega"] = tar_omega
            self._target["reset_flag"] = (
                joy_state.btn_b and not self.__previous_buttons["b"])
            self._target["quit_flag"] = (
                joy_state.btn_x and not self.__previous_buttons["x"])
            self._target["grip_flag"] = bool(joy_state.btn_a)
            self.__previous_buttons.update({
                "a": bool(joy_state.btn_a),
                "b": bool(joy_state.btn_b),
                "x": bool(joy_state.btn_x),
            })

    def __reset_target(self) -> None:
        """Reset the shared target pose from the initial joint position."""
        initial_pose = self.__dyn_util.forward_kinematics(
            self.__init_joint)[-1]
        with self._target_lock:
            self._target["tar_pose"] = (
                np.asarray(initial_pose[0], dtype=np.float64).copy(),
                np.asarray(initial_pose[1], dtype=np.float64).copy(),
            )
            self._target["tar_vel"] = np.zeros(3, dtype=np.float64)
            self._target["tar_omega"] = np.zeros(3, dtype=np.float64)
            self._target["reset_flag"] = False

    def __constrain_target_position(
            self, tar_pose: Tuple[np.ndarray, np.ndarray]) -> np.ndarray:
        """Constrain a target pose using the stable-point radial limits.

        The target end pose is converted to its stable point, whose distance
        from the base origin is clamped to the configured workspace. The
        constrained stable point is then converted back to an end position.

        Args:
            tar_pose: Target end position and wxyz quaternion.

        Returns:
            Target end position after applying the workspace constraint.
        """
        trans_end_in_base = part2trans(tar_pose[0], tar_pose[1])
        trans_stable_in_base = (
            trans_end_in_base @ self.__trans_stable_in_end)
        stable_pos = trans_stable_in_base[:3, 3]
        stable_distance = np.linalg.norm(stable_pos)

        if stable_distance < 1e-9:
            stable_pos = np.array(
                [0.0, 0.0, self.__stable_distance_min],
                dtype=np.float64,
            )
            stable_distance = self.__stable_distance_min
        constrained_distance = np.clip(
            stable_distance,
            self.__stable_distance_min,
            self.__stable_distance_max,
        )
        trans_stable_in_base[:3, 3] = (
            stable_pos / stable_distance * constrained_distance)
        constrained_pose = trans2part(
            trans_stable_in_base @ self.__trans_end_in_stable)
        return np.asarray(constrained_pose[0], dtype=np.float64)

    def __publish_target_marker(self) -> None:
        """Publish a marker from a consistent target snapshot."""
        target = self.__copy_target()
        constrained_tar_pos = self.__constrain_target_position(
            target["tar_pose"])
        self.__data_interface.pub_target_marker_array(
            constrained_tar_pos,
            target["tar_pose"][1],
        )

    ##############################################################
    # Processes
    ##############################################################
    def __init_process(self) -> None:
        """Move the arm to the configured initial joint position."""
        self.__data_interface.logi(
            "[joy_arm] moving to initial joint position")
        grip_jnt_pos = np.zeros(GRIP_DOF, dtype=np.float64)

        while self.__is_running():
            state = self.__data_interface.get_manip_state(latest=True)
            if state is not None:
                jnt_pos = np.asarray(
                    state.manip_state.arm_state.jnt.position,
                    dtype=np.float64,
                )
                tar_jnt_pos, is_interpolating = self.__interpolate_joint(
                    jnt_pos,
                    self.__init_joint,
                    self.__joint_error_limit,
                )
                stable_ctrl = self.__build_stable_ctrl(
                    tar_jnt_pos,
                    grip_jnt_pos,
                )
                self.__data_interface.pub_manip_ctrl(stable_ctrl)

                if not is_interpolating:
                    self.__reset_target()
                    self.__last_arm_command = self.__init_joint.copy()
                    self.__publish_target_marker()
                    self.__data_interface.logi(
                        "[joy_arm] initial position reached; entering work "
                        "mode")
                    return
            self.__data_interface.sleep()

    def __work_process(self) -> None:
        """Run target updates, interpolation, IK, and command publishing."""
        self.__data_interface.logi("[joy_arm] start work mode")

        while self.__is_running():
            state = self.__data_interface.get_manip_state(latest=True)
            joy_state = self.__data_interface.get_joy_state(latest=True)
            if joy_state is not None:
                self.__update_target(joy_state)

            if state is not None:
                jnt_pos = np.asarray(
                    state.manip_state.arm_state.jnt.position,
                    dtype=np.float64,
                )
                target = self.__copy_target()

                if target["quit_flag"]:
                    self.__data_interface.logi("[joy_arm] quit requested")
                    self.__stop_event.set()
                    break

                if target["reset_flag"]:
                    self.__reset_target()
                    self.__reset_active = True
                    self.__data_interface.logi("[joy_arm] target reset")

                target = self.__copy_target()
                grip_jnt_pos = np.array([
                    self.__grip_max_pos
                    if self.__use_gripper and target["grip_flag"] else 0.0
                ], dtype=np.float64)

                if self.__reset_active:
                    tar_jnt_pos, is_interpolating = self.__interpolate_joint(
                        jnt_pos,
                        self.__init_joint,
                        self.__joint_error_limit,
                    )
                    stable_ctrl = self.__build_stable_ctrl(
                        tar_jnt_pos,
                        grip_jnt_pos,
                    )
                    self.__data_interface.pub_manip_ctrl(stable_ctrl)
                    self.__last_arm_command = tar_jnt_pos.copy()

                    if not is_interpolating:
                        self.__reset_active = False
                        self.__last_arm_command = self.__init_joint.copy()
                        self.__data_interface.logi(
                            "[joy_arm] reset position reached")
                else:
                    pos = np.array([
                        state.manip_state.arm_state.pose.position.x,
                        state.manip_state.arm_state.pose.position.y,
                        state.manip_state.arm_state.pose.position.z,
                    ], dtype=np.float64)
                    constrained_tar_pos = self.__constrain_target_position(
                        target["tar_pose"])
                    tar_pos = self.__interpolate_position(
                        pos,
                        constrained_tar_pos,
                        self.__position_error_threshold,
                    )
                    ik_success, tar_jnt_pos = (
                        self.__dyn_util.inverse_kinematics_analytic(
                            (tar_pos, target["tar_pose"][1]),
                            jnt_pos,
                        ))
                    if ik_success:
                        self.__last_arm_command = np.asarray(
                            tar_jnt_pos, dtype=np.float64)
                    else:
                        self.__data_interface.logw(
                            "[joy_arm] inverse kinematics failed")

                    impedance_ctrl = self.__build_impedance_ctrl(
                        self.__last_arm_command,
                        grip_jnt_pos,
                    )
                    self.__data_interface.pub_manip_ctrl(impedance_ctrl)

                self.__publish_target_marker()

            self.__data_interface.sleep()


def main() -> None:
    """Create and run the joystick arm node."""
    joy_arm = JoyArm()
    try:
        joy_arm.start()
        joy_arm.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
