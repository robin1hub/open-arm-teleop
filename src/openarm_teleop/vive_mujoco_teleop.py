#!/usr/bin/env python3
"""Clutched VIVE-controller teleoperation for the OpenArm 1.0 MuJoCo model.

This entry point is simulation-only.  It imports no CAN or OpenArm hardware
driver.  Select an arm with 1/2, then hold that hand controller's grip button
to move the corresponding MuJoCo end-effector target.
"""

from __future__ import annotations

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np
import openvr

from .interactive_mujoco_ee_drag import NativeSingleWindowApp, quat_to_matrix


ROLE_TO_SIDE = {
    openvr.TrackedControllerRole_LeftHand: "left",
    openvr.TrackedControllerRole_RightHand: "right",
}

OPENVR_TO_ROBOT_BASIS = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


def openvr_to_robot_delta(delta: np.ndarray) -> np.ndarray:
    """Map OpenVR standing axes to the OpenArm MuJoCo world axes.

    OpenVR is +X right, +Y up, -Z forward.  The v1 MuJoCo model is +X
    forward, +Y left, +Z up.
    """

    return OPENVR_TO_ROBOT_BASIS @ np.asarray(delta, dtype=np.float64)


def rotation_to_quat(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to MuJoCo's (w, x, y, z) quaternion."""

    matrix = np.asarray(rotation, dtype=np.float64)
    quat = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, matrix.reshape(-1))
    if quat[0] < 0.0:
        quat *= -1.0
    return quat / np.linalg.norm(quat)


class ViveSimulationApp(NativeSingleWindowApp):
    def __init__(
        self,
        side: str,
        scale: float,
        max_offset: float,
        elbow_bend_deg: float,
        position_step_mm: float,
        orientation_step_deg: float,
        gripper_step_mm: float,
    ) -> None:
        super().__init__(side=side, model_version="v1", hardware=None)
        for arm_side in ("right", "left"):
            joints, gripper = self.resolver.get_driver(self.data.qpos, arm_side)
            ready = np.r_[joints, gripper].astype(np.float64)
            ready[3] = np.deg2rad(elbow_bend_deg)
            self.resolver.set_qpos(self.data.qpos, ready, arm_side)
        mujoco.mj_forward(self.model, self.data)
        self._sync_from_model()
        for arm_side in ("right", "left"):
            self._set_mocap(arm_side, self._fk(arm_side))
        self.target = self._mocap_pose(self.side)
        self.ik_target = self.target.copy()
        self._sync_control_sliders()
        self.scale = float(scale)
        self.max_offset = float(max_offset)
        self.vr = openvr.init(openvr.VRApplication_Background)
        self.grip_mask = 1 << openvr.k_EButton_Grip
        self.touchpad_mask = 1 << openvr.k_EButton_SteamVR_Touchpad
        self.gripper_step = gripper_step_mm / 1000.0
        self.finger_qpos = {
            arm_side: [
                int(
                    self.model.jnt_qposadr[
                        mujoco.mj_name2id(
                            self.model,
                            mujoco.mjtObj.mjOBJ_JOINT,
                            f"openarm_{arm_side}_finger_joint{finger}",
                        )
                    ]
                )
                for finger in (1, 2)
            ]
            for arm_side in ("left", "right")
        }
        self.gripper_position = {
            arm_side: float(
                np.mean(
                    [
                        self.data.qpos[address]
                        for address in self.finger_qpos[arm_side]
                    ]
                )
            )
            for arm_side in ("left", "right")
        }
        self.gripper_action = {"left": "HOLD", "right": "HOLD"}
        self.controller_indices: dict[str, int] = {}
        self.anchor_controller: dict[str, np.ndarray] = {}
        self.anchor_controller_rotation: dict[str, np.ndarray] = {}
        self.anchor_robot: dict[str, np.ndarray] = {}
        self.latest_controller: dict[str, np.ndarray] = {}
        self.latest_controller_rotation: dict[str, np.ndarray] = {}
        self.grip_active = {"left": False, "right": False}
        self.require_grip_release = {"left": False, "right": False}
        self.tracking_valid = {"left": False, "right": False}
        self.controller_grip_pressed = {"left": False, "right": False}
        self.last_vr_log = 0.0
        # Simulation-only Cartesian rate limits.  Physical-arm limits must be
        # configured separately at the joint-command/safety layer.
        self.target_position_step = position_step_mm / 1000.0
        self.target_orientation_step = np.deg2rad(orientation_step_deg)
        self.strict_orientation_cost = 0.45
        self.vive_ik_targets = {
            arm_side: self._mocap_pose(arm_side).copy()
            for arm_side in ("left", "right")
        }
        self._discover_controllers()

    def _discover_controllers(self) -> None:
        found: dict[str, int] = {}
        for index in range(openvr.k_unMaxTrackedDeviceCount):
            if (
                self.vr.getTrackedDeviceClass(index)
                != openvr.TrackedDeviceClass_Controller
            ):
                continue
            role = self.vr.getControllerRoleForTrackedDeviceIndex(index)
            side = ROLE_TO_SIDE.get(role)
            if side is not None:
                found[side] = index
        self.controller_indices = found

    @staticmethod
    def _position(pose: object) -> np.ndarray:
        matrix = pose.mDeviceToAbsoluteTracking
        return np.array(
            [matrix[0][3], matrix[1][3], matrix[2][3]], dtype=np.float64
        )

    @staticmethod
    def _rotation(pose: object) -> np.ndarray:
        matrix = pose.mDeviceToAbsoluteTracking
        return np.array(
            [[matrix[row][column] for column in range(3)] for row in range(3)],
            dtype=np.float64,
        )

    def _grip_pressed(self, index: int) -> bool:
        success, state = self.vr.getControllerState(index)
        return bool(success and (state.ulButtonPressed & self.grip_mask))

    def _update_gripper(self, side: str, state: object) -> str:
        """Use the clicked upper/lower touchpad to open/close one gripper."""

        if not (state.ulButtonPressed & self.touchpad_mask):
            return "HOLD"
        vertical = float(state.rAxis[0].y)
        if vertical > 0.25:
            delta = self.gripper_step
            action = "OPEN"
        elif vertical < -0.25:
            delta = -self.gripper_step
            action = "CLOSE"
        else:
            return "HOLD"
        self.gripper_position[side] = float(
            np.clip(self.gripper_position[side] + delta, 0.0, 0.044)
        )
        return action

    def _apply_gripper_positions(self) -> None:
        """Write persistent gripper commands after IK has updated arm qpos."""

        for side in ("left", "right"):
            for qpos_address in self.finger_qpos[side]:
                self.data.qpos[qpos_address] = self.gripper_position[side]
        mujoco.mj_forward(self.model, self.data)

    def update_vive_target(self) -> None:
        self._discover_controllers()
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0.0,
            openvr.k_unMaxTrackedDeviceCount,
        )
        states: dict[str, tuple[bool, bool, float, str]] = {}
        for arm_side in ("left", "right"):
            index = self.controller_indices.get(arm_side)
            controller_ok = False
            controller_state = None
            if index is not None and poses[index].bDeviceIsConnected:
                controller_ok, controller_state = self.vr.getControllerState(index)
            valid = bool(
                index is not None
                and poses[index].bDeviceIsConnected
                and poses[index].bPoseIsValid
                and poses[index].eTrackingResult
                == openvr.TrackingResult_Running_OK
            )
            raw_pressed = bool(
                controller_ok
                and (controller_state.ulButtonPressed & self.grip_mask)
            )
            self.tracking_valid[arm_side] = valid
            self.controller_grip_pressed[arm_side] = raw_pressed
            gripper_action = (
                self._update_gripper(arm_side, controller_state)
                if controller_ok
                else "HOLD"
            )
            self.gripper_action[arm_side] = gripper_action
            if not raw_pressed:
                self.require_grip_release[arm_side] = False
            pressed = bool(
                valid
                and raw_pressed
                and not self.require_grip_release[arm_side]
            )
            if valid:
                self.latest_controller[arm_side] = self._position(poses[index])
                self.latest_controller_rotation[arm_side] = self._rotation(poses[index])

            if pressed and not self.grip_active[arm_side]:
                self.anchor_controller[arm_side] = self._position(poses[index])
                self.anchor_controller_rotation[arm_side] = self._rotation(poses[index])
                self.anchor_robot[arm_side] = self._mocap_pose(arm_side).copy()
                self.grip_active[arm_side] = True
                self.side = arm_side
                self._update_target_colors()
                print(f"[vive] {arm_side} clutch ON", flush=True)
            elif not pressed and self.grip_active[arm_side]:
                self.grip_active[arm_side] = False
                if raw_pressed:
                    self.require_grip_release[arm_side] = True
                print(f"[vive] {arm_side} clutch OFF", flush=True)

            offset_norm = 0.0
            if self.grip_active[arm_side]:
                controller_delta = (
                    self._position(poses[index]) - self.anchor_controller[arm_side]
                )
                robot_delta = self.scale * openvr_to_robot_delta(controller_delta)
                offset_norm = float(np.linalg.norm(robot_delta))
                if offset_norm > self.max_offset:
                    robot_delta *= self.max_offset / offset_norm
                    offset_norm = self.max_offset
                target = self.anchor_robot[arm_side].copy()
                target[:3] += robot_delta
                controller_rotation_delta = (
                    self._rotation(poses[index])
                    @ self.anchor_controller_rotation[arm_side].T
                )
                robot_rotation_delta = (
                    OPENVR_TO_ROBOT_BASIS
                    @ controller_rotation_delta
                    @ OPENVR_TO_ROBOT_BASIS.T
                )
                target[3:] = rotation_to_quat(
                    robot_rotation_delta
                    @ quat_to_matrix(self.anchor_robot[arm_side][3:])
                )
                self.orientation_strict_until = time.monotonic() + 0.2
                self._set_mocap(arm_side, target)
            states[arm_side] = (
                valid,
                self.grip_active[arm_side],
                offset_norm,
                gripper_action,
            )

        now = time.monotonic()
        if now - self.last_vr_log >= 1.0:
            details = " ".join(
                f"{arm_side}={'TRACKING' if valid else 'NO-POSE'}/"
                f"{'ON' if clutch else 'OFF'}/{offset * 1000:.0f}mm/"
                f"gripper={gripper}:{self.gripper_position[arm_side] * 1000:.1f}mm"
                for arm_side, (valid, clutch, offset, gripper) in states.items()
            )
            print(f"[vive] {details}", flush=True)
            self.last_vr_log = now

    def _auto_reset_vive_target(self, side: str) -> None:
        """Apply the R reset and re-clutch in place while grip remains held."""

        pose = self._fk(side)
        self._sync_from_model()
        self._set_mocap(side, pose)
        self.target = pose.copy()
        self.ik_target = pose.copy()
        self.reachable = True
        self.position_error = 0.0
        self.orientation_error = 0.0
        self.last_reject_reason = ""

        # Make the current controller pose the new relative origin.  Thus the
        # next valid sample continues smoothly instead of recreating the same
        # unreachable target or requiring a grip release.
        if (
            self.grip_active[side]
            and side in self.latest_controller
            and side in self.latest_controller_rotation
        ):
            self.anchor_controller[side] = self.latest_controller[side].copy()
            self.anchor_controller_rotation[side] = (
                self.latest_controller_rotation[side].copy()
            )
            self.anchor_robot[side] = pose.copy()
        print(
            f"[vive] {side} target rejected; auto-R reset "
            f"(clutch remains {'ON' if self.grip_active[side] else 'OFF'})",
            flush=True,
        )

    def run(self) -> None:
        print(
            "VIVE SIMULATION ONLY: 1=left, 2=right, hold matching grip to move, "
            "release to freeze, R=reset, Space=pause, Esc=quit.",
            flush=True,
        )
        try:
            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                key_callback=self._native_key_callback,
                show_left_ui=True,
                show_right_ui=True,
            ) as viewer:
                viewer.cam.azimuth = 135.0
                viewer.cam.elevation = -18.0
                viewer.cam.distance = 1.7
                while viewer.is_running():
                    start = time.monotonic()
                    with viewer.lock():
                        self._handle_native_keys(viewer)
                        self._apply_control_sliders()
                        self.update_vive_target()
                        displayed_side = self.side
                        for arm_side in ("left", "right"):
                            self.side = arm_side
                            self.target = self._mocap_pose(arm_side)
                            self.ik_target = self.vive_ik_targets[arm_side]
                            self.solve_once()
                            self.vive_ik_targets[arm_side] = self.ik_target.copy()
                            if not self.reachable:
                                self._auto_reset_vive_target(arm_side)
                                self.vive_ik_targets[arm_side] = self.ik_target.copy()
                        self._apply_gripper_positions()
                        self.after_solve()
                        self.side = displayed_side
                        self.target = self._mocap_pose(self.side)
                        self.ik_target = self.vive_ik_targets[self.side]
                        self._sync_control_sliders()
                    if self.close_requested:
                        viewer.close()
                        continue
                    viewer.sync()
                    time.sleep(max(0.0, 1.0 / 60.0 - (time.monotonic() - start)))
        finally:
            openvr.shutdown()

    def after_solve(self) -> None:
        """Hook for an explicitly selected physical-control subclass."""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="robot displacement / controller displacement (default: 1.0)",
    )
    parser.add_argument(
        "--max-offset",
        type=float,
        default=0.25,
        help="maximum displacement during one grip hold, metres (default: 0.25)",
    )
    parser.add_argument(
        "--elbow-bend-deg",
        type=float,
        default=45.0,
        help="simulation-only initial J4 bend for better IK conditioning (default: 45)",
    )
    parser.add_argument(
        "--position-step-mm",
        type=float,
        default=10.0,
        help="simulation-only target movement per 60 Hz frame (default: 10 mm)",
    )
    parser.add_argument(
        "--orientation-step-deg",
        type=float,
        default=2.0,
        help="simulation-only target rotation per 60 Hz frame (default: 2 degrees)",
    )
    parser.add_argument(
        "--gripper-step-mm",
        type=float,
        default=1.0,
        help="simulation-only gripper movement per 60 Hz frame (default: 1 mm)",
    )
    args = parser.parse_args()
    if not 0.05 <= args.scale <= 2.0:
        parser.error("--scale must be between 0.05 and 2.0")
    if not 0.02 <= args.max_offset <= 0.5:
        parser.error("--max-offset must be between 0.02 and 0.5")
    if not 10.0 <= args.elbow_bend_deg <= 100.0:
        parser.error("--elbow-bend-deg must be between 10 and 100")
    if not 1.0 <= args.position_step_mm <= 20.0:
        parser.error("--position-step-mm must be between 1 and 20")
    if not 0.25 <= args.orientation_step_deg <= 5.0:
        parser.error("--orientation-step-deg must be between 0.25 and 5")
    if not 0.1 <= args.gripper_step_mm <= 2.0:
        parser.error("--gripper-step-mm must be between 0.1 and 2")
    ViveSimulationApp(
        args.side,
        args.scale,
        args.max_offset,
        args.elbow_bend_deg,
        args.position_step_mm,
        args.orientation_step_deg,
        args.gripper_step_mm,
    ).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
