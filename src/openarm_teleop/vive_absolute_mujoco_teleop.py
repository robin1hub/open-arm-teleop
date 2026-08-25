#!/usr/bin/env python3
"""Absolute-body-frame VIVE teleoperation for OpenArm 1.0 MuJoCo.

This is a separate simulation-only controller.  It preserves the original
clutched relative controller in vive_mujoco_teleop.py.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
import openvr

from .interactive_mujoco_ee_drag import (
    add_geom,
    add_target_frame,
    approach_quat,
    quat_error_deg,
    quat_to_matrix,
)
from .vive_mujoco_teleop import (
    ViveSimulationApp,
    rotation_to_quat,
)

# Columns express the OpenArm TCP +X/+Y/+Z axes in VIVE tip coordinates:
# gripper +X <- controller +Y
# gripper +Y <- controller +X
# gripper +Z <- controller -Z (the wand's pointing direction)
CONTROLLER_TIP_TO_GRIPPER_AXES = np.array(
    [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
    dtype=np.float64,
)


def body_basis_from_hmd(rotation: np.ndarray) -> np.ndarray:
    """Return world-from-body basis with columns forward, left and up."""

    rotation = np.asarray(rotation, dtype=np.float64)
    forward = np.array([-rotation[0, 2], 0.0, -rotation[2, 2]])
    norm = float(np.linalg.norm(forward))
    if norm < 0.2:
        raise ValueError("HMD forward direction is too vertical for calibration")
    forward /= norm
    up = np.array([0.0, 1.0, 0.0])
    left = np.cross(up, forward)
    left /= np.linalg.norm(left)
    return np.column_stack((forward, left, up))


def map_absolute_pose(
    hand_position_world: np.ndarray,
    hand_rotation_world: np.ndarray,
    body_origin_world: np.ndarray,
    body_basis_world: np.ndarray,
    calibrated_hand_position_body: np.ndarray,
    calibrated_hand_rotation_body: np.ndarray,
    robot_anchor: np.ndarray,
    scale_xyz: np.ndarray,
) -> np.ndarray:
    """Map a tracked hand pose into the calibrated robot body frame."""

    hand_position_body = body_basis_world.T @ (
        np.asarray(hand_position_world) - body_origin_world
    )
    hand_rotation_body = body_basis_world.T @ np.asarray(hand_rotation_world)
    position_delta = scale_xyz * (
        hand_position_body - calibrated_hand_position_body
    )
    rotation_delta = hand_rotation_body @ calibrated_hand_rotation_body.T
    target = np.asarray(robot_anchor, dtype=np.float64).copy()
    target[:3] += position_delta
    target[3:] = rotation_to_quat(
        rotation_delta @ quat_to_matrix(robot_anchor[3:])
    )
    return target


class ViveAbsoluteSimulationApp(ViveSimulationApp):
    """Fixed-calibration body mapping with continuous safe target projection."""

    def __init__(
        self,
        scale_xyz: np.ndarray,
        shoulder_width: float,
        shoulder_drop: float,
        mapping_mode: str,
        **kwargs: object,
    ) -> None:
        super().__init__(**kwargs)
        self.manual_scale_xyz = np.asarray(scale_xyz, dtype=np.float64)
        self.scale_xyz = self.manual_scale_xyz.copy()
        self.shoulder_width = float(shoulder_width)
        self.shoulder_drop = float(shoulder_drop)
        self.mapping_mode = mapping_mode
        self.shared_translation = np.zeros(3, dtype=np.float64)
        self.render_models = openvr.VRRenderModels()
        self.controller_tip_transforms: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        # Absolute hand orientation is a primary command, not a secondary
        # visual preference.  Keep position and orientation comparably strong.
        self.strict_orientation_cost = 1.0
        self.calibrated = False
        self.calibration_stage = 0
        self.calibration_session_active = True
        self.calibration_trigger_latched = False
        self.calibration_samples: dict[str, object] = {}
        self.human_arm_length: dict[str, float] = {}
        self.human_upper_arm: dict[str, float] = {}
        self.human_forearm: dict[str, float] = {}
        self.shoulder_position_body: dict[str, np.ndarray] = {}
        self.body_origin_world = np.zeros(3)
        self.body_basis_world = np.eye(3)
        self.calibrated_hand_position_body: dict[str, np.ndarray] = {}
        self.calibrated_hand_rotation_body: dict[str, np.ndarray] = {}
        self.robot_calibration_anchor: dict[str, np.ndarray] = {}
        self.raw_targets = {
            side: self._fk(side).copy() for side in ("left", "right")
        }
        self.safe_targets = {
            side: self._fk(side).copy() for side in ("left", "right")
        }
        self.projected = {"left": False, "right": False}
        self.last_projection_reason = {"left": "", "right": ""}
        self.last_tracking_frame: dict[str, object] | None = None
        self.solve_order_flipped = False
        self.home_qpos = self.data.qpos.copy()
        self.home_reset_active = False
        self.home_reset_goal = self.home_qpos.copy()
        self.home_joint_step = np.deg2rad(1.0)
        self.collision_data = mujoco.MjData(self.model)
        self.trigger_mask = 1 << openvr.k_EButton_SteamVR_Trigger
        self.vr_overlay = None
        self.vr_notifications = None
        self.notification_overlay_handle = None
        self.notification_ids: list[int] = []
        self._initialize_headset_guidance()
        self.calibration_ui = self._launch_calibration_ui()
        self.last_calibration_ui_update = 0.0
        step_description = (
            "1=facing, 2=arms down, 3=T-pose"
            if self.mapping_mode == "shared"
            else "1=facing, 2=arms down, 3=T-pose, 4=elbows 90deg forward"
        )
        print(
            f"[calibration] mode={self.mapping_mode}. K starts/restarts; "
            f"controller TRIGGER (or keyboard C) captures: "
            f"{step_description}. Release the trigger between poses.",
            flush=True,
        )
        total = self._calibration_step_count()
        self._notify_headset(
            f"OpenArm 标定 1/{total}\n面向正前方，按一下扳机"
        )

    def _initialize_headset_guidance(self) -> None:
        try:
            self.vr_overlay = openvr.VROverlay()
            self.vr_notifications = openvr.VRNotifications()
            self.notification_overlay_handle = self.vr_overlay.createOverlay(
                "dev.openarm.vive.calibration",
                "OpenArm VIVE Calibration",
            )
        except Exception as error:
            print(f"[headset-guide] unavailable: {error}", flush=True)
            self.vr_overlay = None
            self.vr_notifications = None
            self.notification_overlay_handle = None

    def _notify_headset(self, text: str) -> None:
        if (
            self.vr_notifications is None
            or self.notification_overlay_handle is None
        ):
            return
        try:
            notification_id = self.vr_notifications.createNotification(
                self.notification_overlay_handle,
                0,
                openvr.EVRNotificationType_Transient,
                text[: openvr.k_unNotificationTextMaxSize - 1],
                openvr.EVRNotificationStyle_Application,
                openvr.NotificationBitmap_t(),
            )
            self.notification_ids.append(int(notification_id))
        except Exception as error:
            # pyopenvr 1.26 incorrectly raises its generated OK exception even
            # though SteamVR accepted the notification.
            if type(error).__name__ != "NotificationError_OK":
                print(f"[headset-guide] notification failed: {error}", flush=True)

    @staticmethod
    def _launch_calibration_ui() -> subprocess.Popen[str] | None:
        helper = pathlib.Path(__file__).with_name("vive_calibration_status.py")
        try:
            return subprocess.Popen(
                [sys.executable, str(helper)],
                stdin=subprocess.PIPE,
                text=True,
            )
        except OSError as error:
            print(f"[calibration-ui] unavailable: {error}", flush=True)
            return None

    def _update_calibration_ui(
        self, frame: dict[str, object], force: bool = False
    ) -> None:
        now = time.monotonic()
        if not force and now - self.last_calibration_ui_update < 0.15:
            return
        process = self.calibration_ui
        if process is None or process.poll() is not None or process.stdin is None:
            return
        hands = frame["hands"]
        payload = {
            "completed": (
                self._calibration_step_count()
                if self.calibrated
                else self.calibration_stage
            ),
            "total_steps": self._calibration_step_count(),
            "mapping_mode": self.mapping_mode,
            "calibrated": self.calibrated,
            "tracking": {
                "hmd": bool(frame["hmd_valid"]),
                "left": bool(hands["left"]["valid"]),
                "right": bool(hands["right"]["valid"]),
            },
        }
        try:
            process.stdin.write(json.dumps(payload) + "\n")
            process.stdin.flush()
            self.last_calibration_ui_update = now
        except (BrokenPipeError, OSError):
            self.calibration_ui = None

    def _calibration_step_count(self) -> int:
        return 3 if self.mapping_mode == "shared" else 4

    def _close_calibration_ui(self) -> None:
        process = self.calibration_ui
        if process is None:
            return
        if process.stdin is not None:
            process.stdin.close()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.terminate()

    def _close_headset_guidance(self) -> None:
        if self.vr_notifications is not None:
            for notification_id in self.notification_ids:
                try:
                    self.vr_notifications.removeNotification(notification_id)
                except Exception:
                    pass
        if self.vr_overlay is not None and self.notification_overlay_handle is not None:
            try:
                self.vr_overlay.destroyOverlay(self.notification_overlay_handle)
            except Exception:
                pass

    def _sample_tracking(self) -> dict[str, object]:
        self._discover_controllers()
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0.0,
            openvr.k_unMaxTrackedDeviceCount,
        )
        hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
        hmd_valid = bool(
            hmd_pose.bDeviceIsConnected
            and hmd_pose.bPoseIsValid
            and hmd_pose.eTrackingResult == openvr.TrackingResult_Running_OK
        )
        hands: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            index = self.controller_indices.get(side)
            controller_ok = False
            state = None
            if index is not None and poses[index].bDeviceIsConnected:
                controller_ok, state = self.vr.getControllerState(index)
            valid = bool(
                index is not None
                and poses[index].bDeviceIsConnected
                and poses[index].bPoseIsValid
                and poses[index].eTrackingResult
                == openvr.TrackingResult_Running_OK
            )
            grip = bool(
                controller_ok and state.ulButtonPressed & self.grip_mask
            )
            if valid:
                position, rotation = self._controller_tip_pose(
                    index, poses[index], state
                )
            else:
                position, rotation = None, None
            hands[side] = {
                "valid": valid,
                "position": position,
                "rotation": rotation,
                "grip": grip,
                "controller_ok": controller_ok,
                "state": state,
            }
        frame: dict[str, object] = {
            "hmd_valid": hmd_valid,
            "hmd_position": self._position(hmd_pose) if hmd_valid else None,
            "hmd_rotation": self._rotation(hmd_pose) if hmd_valid else None,
            "hands": hands,
        }
        self.last_tracking_frame = frame
        return frame

    def _controller_tip_pose(
        self, index: int, pose: object, state: object
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return the render-model tip pose instead of the tracking origin."""

        if index not in self.controller_tip_transforms:
            translation = np.zeros(3, dtype=np.float64)
            rotation = np.eye(3, dtype=np.float64)
            try:
                model_name = self.vr.getStringTrackedDeviceProperty(
                    index, openvr.Prop_RenderModelName_String
                )
                mode = openvr.RenderModel_ControllerMode_State_t()
                success, component = self.render_models.getComponentState(
                    model_name, "tip", state, mode
                )
                if success:
                    matrix = component.mTrackingToComponentLocal
                    translation = np.array(
                        [matrix[row][3] for row in range(3)], dtype=np.float64
                    )
                    rotation = np.array(
                        [
                            [matrix[row][column] for column in range(3)]
                            for row in range(3)
                        ],
                        dtype=np.float64,
                    )
                    print(
                        f"[controller-tip] device={index} model={model_name} "
                        f"offset_mm={(translation * 1000).round(1).tolist()}",
                        flush=True,
                    )
            except Exception as error:
                print(
                    f"[controller-tip] device={index} fallback to tracking "
                    f"origin: {error}",
                    flush=True,
                )
            self.controller_tip_transforms[index] = (translation, rotation)
        tip_translation, tip_rotation = self.controller_tip_transforms[index]
        tracking_position = self._position(pose)
        tracking_rotation = self._rotation(pose)
        return (
            tracking_position + tracking_rotation @ tip_translation,
            tracking_rotation @ tip_rotation,
        )

    @staticmethod
    def _valid_calibration_frame(frame: dict[str, object]) -> bool:
        hands = frame["hands"]
        return bool(
            frame["hmd_valid"]
            and all(hands[side]["valid"] for side in ("left", "right"))
        )

    def _hand_body(
        self, frame: dict[str, object], side: str
    ) -> tuple[np.ndarray, np.ndarray]:
        hand = frame["hands"][side]
        position = self.body_basis_world.T @ (
            np.asarray(hand["position"], dtype=np.float64)
            - self.body_origin_world
        )
        rotation = self.body_basis_world.T @ np.asarray(
            hand["rotation"], dtype=np.float64
        )
        return position, rotation

    def _shoulder_body(self, side: str) -> np.ndarray:
        if side in self.shoulder_position_body:
            return self.shoulder_position_body[side]
        lateral = self.shoulder_width / 2.0
        return np.array(
            [0.0, lateral if side == "left" else -lateral, -self.shoulder_drop],
            dtype=np.float64,
        )

    def _robot_reach(self, side: str) -> float:
        shoulder_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{side}_link1"
        )
        shoulder = self.data.xpos[shoulder_id]
        return float(np.linalg.norm(self._fk(side)[:3] - shoulder))

    def _finish_anthropometric_calibration(
        self, elbow_frame: dict[str, object]
    ) -> None:
        for side in ("left", "right"):
            elbow_hand, _ = self._hand_body(elbow_frame, side)
            shoulder = self._shoulder_body(side)
            total = self.human_arm_length[side]
            diagonal = float(np.linalg.norm(elbow_hand - shoulder))
            discriminant = 2.0 * diagonal * diagonal - total * total
            if discriminant >= 0.0:
                root = float(np.sqrt(discriminant))
                upper = 0.5 * (total + root)
                forearm = total - upper
                # Human segments are normally close in length.  Reject an
                # implausible "90 degree" sample rather than amplifying it.
                if not (0.35 * total <= forearm <= 0.65 * total):
                    upper, forearm = 0.52 * total, 0.48 * total
            else:
                upper, forearm = 0.52 * total, 0.48 * total
            self.human_upper_arm[side] = upper
            self.human_forearm[side] = forearm

        self.calibrated = True
        self.calibration_stage = 0
        self.calibration_session_active = False
        self._haptic_calibration_complete()
        print(
            "[absolute] CALIBRATED: "
            + " ".join(
                f"{side} arm={self.human_arm_length[side]:.3f}m "
                f"upper/forearm={self.human_upper_arm[side]:.3f}/"
                f"{self.human_forearm[side]:.3f}m"
                for side in ("left", "right")
            )
            + f" scale={self.scale_xyz.round(3).tolist()}",
            flush=True,
        )

    def _finish_shared_calibration(self) -> None:
        """Finish shared mapping after T-pose; no elbow model is required."""

        for side in ("left", "right"):
            total = self.human_arm_length[side]
            # Retain a nominal estimate for diagnostics only.  Shared mode
            # does not feed an inferred human elbow into IK.
            self.human_upper_arm[side] = 0.52 * total
            self.human_forearm[side] = 0.48 * total
            self.projected[side] = False
            self.last_projection_reason[side] = ""
        self.calibrated = True
        self.calibration_stage = 0
        self.calibration_session_active = False
        self._haptic_calibration_complete()
        print(
            "[absolute] CALIBRATED shared 3-step: "
            + " ".join(
                f"{side} arm={self.human_arm_length[side]:.3f}m"
                for side in ("left", "right")
            )
            + f" scale={self.scale_xyz.round(3).tolist()}",
            flush=True,
        )

    def _haptic_calibration_complete(self) -> None:
        """Give both VIVE wands a clear completion pulse."""

        pulsed = []
        for side in ("left", "right"):
            index = self.controller_indices.get(side)
            if index is None:
                continue
            try:
                self.vr.triggerHapticPulse(index, 0, 3999)
                pulsed.append(side)
            except Exception as error:
                print(f"[haptic] {side} pulse failed: {error}", flush=True)
        if pulsed:
            print(f"[haptic] calibration complete pulse: {pulsed}", flush=True)

    def _haptic_stage(self, stage: int) -> None:
        """Pulse both wands after every accepted sample."""

        duration = (900, 1500, 2400, 3999)[max(0, min(stage - 1, 3))]
        for side in ("left", "right"):
            index = self.controller_indices.get(side)
            if index is None:
                continue
            try:
                self.vr.triggerHapticPulse(index, 0, duration)
            except Exception as error:
                print(f"[haptic] {side} stage pulse failed: {error}", flush=True)

    def calibrate_absolute(self) -> bool:
        if not self.calibration_session_active:
            print(
                "[absolute] calibration capture ignored: press K to start "
                "a new calibration",
                flush=True,
            )
            return False
        frame = self._sample_tracking()
        self._update_calibration_ui(frame)
        if not self._valid_calibration_frame(frame):
            print(
                "[absolute] C rejected: HMD and both controllers must be TRACKING",
                flush=True,
            )
            return False
        if any(frame["hands"][side]["grip"] for side in ("left", "right")):
            print(
                "[absolute] C rejected: release both grip buttons during calibration",
                flush=True,
            )
            return False

        if self.calibration_stage == 0:
            try:
                self.body_basis_world = body_basis_from_hmd(
                    frame["hmd_rotation"]
                )
            except ValueError as error:
                print(f"[absolute] C rejected: {error}", flush=True)
                return False
            # Only the HMD yaw is used.  Positions stay in a fixed translated
            # OpenVR frame; all control deltas are relative and translation
            # invariant.  Shoulder positions are recovered from the T-pose.
            self.body_origin_world = np.zeros(3, dtype=np.float64)
            self.calibrated = False
            self.calibration_samples = {"facing": frame}
            self.calibration_stage = 1
            self._haptic_stage(1)
            total = self._calibration_step_count()
            self._notify_headset(
                f"已完成 1/{total}：正视\n双臂自然下垂，然后按一下扳机"
            )
            print(
                f"[calibration 1/{total}] facing captured. Put BOTH ARMS "
                "NATURALLY DOWN, keep wrists neutral, then press trigger.",
                flush=True,
            )
            return True

        if self.calibration_stage == 1:
            self.calibration_samples["down"] = frame
            for side in ("left", "right"):
                position, rotation = self._hand_body(frame, side)
                self.calibrated_hand_position_body[side] = position
                self.calibrated_hand_rotation_body[side] = rotation
                anchor = self._fk(side).copy()
                self.robot_calibration_anchor[side] = anchor
                self.raw_targets[side] = anchor.copy()
                self.safe_targets[side] = anchor.copy()
                self.vive_ik_targets[side] = anchor.copy()
                self._set_mocap(side, anchor)
            self.calibration_stage = 2
            self._haptic_stage(2)
            total = self._calibration_step_count()
            self._notify_headset(
                f"已完成 2/{total}：双臂下垂\n双臂左右平举做 T-Pose，然后按扳机"
            )
            captured_detail = (
                "shared translation reference captured"
                if self.mapping_mode == "shared"
                else "controller-to-gripper orientation captured"
            )
            print(
                f"[calibration 2/{total}] arms-down pose and "
                f"{captured_detail}. Make a straight horizontal T-POSE, "
                "then press trigger.",
                flush=True,
            )
            return True

        if self.calibration_stage == 2:
            self.calibration_samples["t_pose"] = frame
            t_positions = {
                side: self._hand_body(frame, side)[0]
                for side in ("left", "right")
            }
            hand_midpoint = 0.5 * (
                t_positions["left"] + t_positions["right"]
            )
            lateral_span = float(
                t_positions["left"][1] - t_positions["right"][1]
            )
            if not 0.95 <= lateral_span <= 2.20:
                print(
                    f"[calibration] C rejected: T-pose hand span "
                    f"{lateral_span:.3f}m is implausible",
                    flush=True,
                )
                return False
            reach_from_span = 0.5 * (lateral_span - self.shoulder_width)
            if not 0.35 <= reach_from_span <= 1.0:
                print(
                    f"[calibration] C rejected: arm reach from T-pose "
                    f"{reach_from_span:.3f}m is implausible",
                    flush=True,
                )
                return False
            self.shoulder_position_body = {
                "left": hand_midpoint
                + np.array([0.0, self.shoulder_width / 2.0, 0.0]),
                "right": hand_midpoint
                + np.array([0.0, -self.shoulder_width / 2.0, 0.0]),
            }
            ratios = []
            for side in ("left", "right"):
                human_reach = float(
                    np.linalg.norm(
                        t_positions[side] - self._shoulder_body(side)
                    )
                )
                if not 0.35 <= human_reach <= 1.0:
                    print(
                        f"[calibration] C rejected: {side} T-pose reach "
                        f"{human_reach:.3f}m is implausible",
                        flush=True,
                    )
                    return False
                self.human_arm_length[side] = human_reach
                ratios.append(self._robot_reach(side) / human_reach)
            automatic_scale = float(np.clip(np.mean(ratios), 0.4, 1.2))
            self.scale_xyz = automatic_scale * self.manual_scale_xyz
            down_frame = self.calibration_samples["down"]
            human_down_midpoint = 0.5 * sum(
                (
                    self._hand_body(down_frame, side)[0]
                    for side in ("left", "right")
                ),
                start=np.zeros(3, dtype=np.float64),
            )
            robot_midpoint = 0.5 * (
                self.robot_calibration_anchor["left"][:3]
                + self.robot_calibration_anchor["right"][:3]
            )
            self.shared_translation = (
                robot_midpoint - self.scale_xyz * human_down_midpoint
            )
            if self.mapping_mode == "shared":
                self._finish_shared_calibration()
                self._notify_headset(
                    "OpenArm 共享标定完成\n可使用侧握键控制机械臂"
                )
                self._update_calibration_ui(frame, force=True)
                return True
            self.calibration_stage = 3
            self._haptic_stage(3)
            self._notify_headset(
                "已完成 3/4：T-Pose\n上臂下垂、前臂向前弯肘 90°，然后按扳机"
            )
            print(
                f"[calibration 3/4] T-pose captured; automatic reach scale="
                f"{automatic_scale:.3f}. Put upper arms down, elbows near "
                "the torso, forearms forward at 90deg, then press C.",
                flush=True,
            )
            return True

        self.calibration_samples["elbow_90"] = frame
        self._finish_anthropometric_calibration(frame)
        self._notify_headset(
            "OpenArm 标定完成\n可将头显挂到脖子上，侧握键控制机械臂"
        )
        self._update_calibration_ui(frame, force=True)
        for side in ("left", "right"):
            self.projected[side] = False
            self.last_projection_reason[side] = ""
        return True

    def _start_calibration_session(self) -> None:
        self.calibrated = False
        self.calibration_stage = 0
        self.calibration_session_active = True
        self.calibration_trigger_latched = True
        self.calibration_samples = {}
        for side in ("left", "right"):
            self.grip_active[side] = False
            self.require_grip_release[side] = True
            self.projected[side] = False
            self.last_projection_reason[side] = ""
        total = self._calibration_step_count()
        print(
            f"[calibration] STARTED with K: 1/{total}=facing, "
            "then use trigger or C to capture each pose",
            flush=True,
        )
        self._notify_headset(
            f"OpenArm 标定 1/{total}\n面向正前方，按一下扳机"
        )

    def _handle_native_keys(self, viewer: object) -> None:
        forwarded: list[int] = []
        while self.pending_keys:
            key = self.pending_keys.pop(0)
            if key in (ord("K"), ord("k")):
                self._start_calibration_session()
            elif key in (ord("C"), ord("c")):
                self.calibrate_absolute()
            elif key in (ord("H"), ord("h")):
                self._begin_home_reset()
            else:
                forwarded.append(key)
        self.pending_keys.extend(forwarded)
        super()._handle_native_keys(viewer)

    def _begin_home_reset(self) -> None:
        """Cancel hand control and start a rate-limited return to ready posture."""

        self.home_reset_goal = self.home_qpos.copy()
        # Preserve the current gripper opening; H resets arm posture only.
        for side in ("left", "right"):
            for address in self.finger_qpos[side]:
                self.home_reset_goal[address] = self.data.qpos[address]
            self.grip_active[side] = False
            # A held grip must be released before it can command motion again.
            self.require_grip_release[side] = True
            self.projected[side] = False
            self.last_projection_reason[side] = ""
        self.home_reset_active = True
        print(
            "[absolute] H HOME requested: hand following cancelled; "
            "returning both arms to the safe initial posture",
            flush=True,
        )

    def _step_home_reset(self) -> None:
        if not self.home_reset_active:
            return
        delta = self.home_reset_goal - self.data.qpos
        if float(np.max(np.abs(delta))) < 1e-4:
            self.data.qpos[:] = self.home_reset_goal
            self.home_reset_active = False
            status = "complete"
        else:
            self.data.qpos[:] += np.clip(
                delta, -self.home_joint_step, self.home_joint_step
            )
            status = ""
        mujoco.mj_forward(self.model, self.data)
        self._sync_from_model()
        self._apply_gripper_positions()
        for side in ("left", "right"):
            pose = self._fk(side).copy()
            self.raw_targets[side] = pose.copy()
            self.safe_targets[side] = pose.copy()
            self.vive_ik_targets[side] = pose.copy()
            self._set_mocap(side, pose)
        if status:
            print(
                "[absolute] H HOME complete; release both grips before "
                "resuming control",
                flush=True,
            )

    def update_vive_target(self) -> None:
        frame = self._sample_tracking()
        self._update_calibration_ui(frame)
        hands = frame["hands"]
        trigger_pressed = any(
            bool(
                hands[side]["controller_ok"]
                and hands[side]["state"].ulButtonPressed & self.trigger_mask
            )
            for side in ("left", "right")
        )
        if (
            self.calibration_session_active
            and trigger_pressed
            and not self.calibration_trigger_latched
        ):
            self.calibration_trigger_latched = True
            self.calibrate_absolute()
        elif not trigger_pressed:
            self.calibration_trigger_latched = False

        for side in ("left", "right"):
            hand = hands[side]
            valid = bool(hand["valid"])
            raw_pressed = bool(hand["grip"])
            self.tracking_valid[side] = valid
            self.controller_grip_pressed[side] = raw_pressed
            if not raw_pressed:
                self.require_grip_release[side] = False
            if hand["controller_ok"]:
                self.gripper_action[side] = self._update_gripper(
                    side, hand["state"]
                )
            else:
                self.gripper_action[side] = "HOLD"

            pressed = bool(
                valid
                and raw_pressed
                and not self.require_grip_release[side]
                and not self.home_reset_active
            )
            if pressed and not self.grip_active[side]:
                self.grip_active[side] = True
                self.side = side
                self._update_target_colors()
                print(f"[absolute] {side} permission ON", flush=True)
            elif not pressed and self.grip_active[side]:
                self.grip_active[side] = False
                print(f"[absolute] {side} permission OFF", flush=True)

            if self.calibrated and valid:
                self.raw_targets[side] = map_absolute_pose(
                    hand["position"],
                    hand["rotation"],
                    self.body_origin_world,
                    self.body_basis_world,
                    self.calibrated_hand_position_body[side],
                    self.calibrated_hand_rotation_body[side],
                    self.robot_calibration_anchor[side],
                    self.scale_xyz,
                )
                if self.mapping_mode == "shared":
                    hand_position_body = self.body_basis_world.T @ (
                        np.asarray(hand["position"], dtype=np.float64)
                        - self.body_origin_world
                    )
                    hand_rotation_body = self.body_basis_world.T @ np.asarray(
                        hand["rotation"], dtype=np.float64
                    )
                    self.raw_targets[side][:3] = (
                        self.shared_translation
                        + self.scale_xyz * hand_position_body
                    )
                    self.raw_targets[side][3:] = rotation_to_quat(
                        hand_rotation_body @ CONTROLLER_TIP_TO_GRIPPER_AXES
                    )

        now = time.monotonic()
        if now - self.last_vr_log >= 1.0:
            details = " ".join(
                f"{side}={'TRACKING' if hands[side]['valid'] else 'NO-POSE'}/"
                f"{'ON' if self.grip_active[side] else 'OFF'}/"
                f"{'PROJECTED' if self.projected[side] else 'DIRECT'}"
                for side in ("left", "right")
            )
            print(
                f"[absolute] calibrated={self.calibrated} "
                f"mode={self.mapping_mode} "
                f"hmd={'TRACKING' if frame['hmd_valid'] else 'NO-POSE'} {details}",
                flush=True,
            )
            self.last_vr_log = now

    @staticmethod
    def _is_finger_pair(name1: str, name2: str) -> bool:
        return (
            "finger_collision" in name1
            and "finger_collision" in name2
            and (
                ("openarm_left_" in name1 and "openarm_left_" in name2)
                or ("openarm_right_" in name1 and "openarm_right_" in name2)
            )
        )

    def _collision_reason(self, data: mujoco.MjData | None = None) -> str:
        checked = self.data if data is None else data
        for contact_index in range(checked.ncon):
            contact = checked.contact[contact_index]
            name1 = (
                mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
                )
                or f"geom{contact.geom1}"
            )
            name2 = (
                mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
                )
                or f"geom{contact.geom2}"
            )
            if self._is_finger_pair(name1, name2):
                continue
            if "openarm_" in name1 or "openarm_" in name2:
                return f"collision: {name1} <-> {name2}"
        return ""

    def _path_collision_reason(
        self, start_qpos: np.ndarray, end_qpos: np.ndarray
    ) -> str:
        """Check the entire small joint-space step, including torso/inter-arm."""

        max_delta = float(np.max(np.abs(end_qpos - start_qpos)))
        samples = max(2, int(np.ceil(max_delta / 0.01)) + 1)
        for sample_index, alpha in enumerate(np.linspace(0.0, 1.0, samples)):
            self.collision_data.qpos[:] = (
                start_qpos + alpha * (end_qpos - start_qpos)
            )
            mujoco.mj_forward(self.model, self.collision_data)
            reason = self._collision_reason(self.collision_data)
            if reason:
                progress = 100.0 * sample_index / (samples - 1)
                return f"{reason} at {progress:.0f}% of step"
        return ""

    def _restore_candidate(
        self, qpos: np.ndarray, ik_target: np.ndarray, side: str
    ) -> None:
        self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)
        self._sync_from_model()
        self.ik_target = ik_target.copy()
        self.vive_ik_targets[side] = ik_target.copy()
        self.reachable = True

    def _attempt_candidate(
        self,
        side: str,
        desired: np.ndarray,
        position_factor: float,
        orientation_factor: float,
    ) -> tuple[bool, str]:
        qpos_before = self.data.qpos.copy()
        ik_before = self.vive_ik_targets[side].copy()
        position_step_before = self.target_position_step
        orientation_step_before = self.target_orientation_step
        self.side = side
        self.target = desired.copy()
        self.ik_target = ik_before.copy()
        self.target_position_step = position_step_before * position_factor
        self.target_orientation_step = (
            orientation_step_before * orientation_factor
        )
        self.orientation_strict_until = time.monotonic() + 0.2
        try:
            self.solve_once()
            self._apply_gripper_positions()
            reason = self.last_reject_reason if not self.reachable else ""
            if not reason:
                reason = self._path_collision_reason(
                    qpos_before, self.data.qpos.copy()
                )
            if reason:
                self._restore_candidate(qpos_before, ik_before, side)
                return False, reason
            self.vive_ik_targets[side] = self.ik_target.copy()
            self.safe_targets[side] = self._fk(side).copy()
            self._set_mocap(side, self.safe_targets[side])
            return True, ""
        except Exception:
            self._restore_candidate(qpos_before, ik_before, side)
            raise
        finally:
            self.target_position_step = position_step_before
            self.target_orientation_step = orientation_step_before

    def solve_projected_side(self, side: str) -> None:
        if not self.calibrated or not self.grip_active[side]:
            return
        raw = self.raw_targets[side]
        safe = self.vive_ik_targets[side]
        attempts: list[tuple[np.ndarray, float, float]] = [
            (raw, 1.0, 1.0),
            (raw, 0.5, 0.5),
            (raw, 0.25, 0.25),
        ]
        delta = raw[:3] - safe[:3]
        for axis in np.argsort(-np.abs(delta)):
            if abs(delta[axis]) < 1e-5:
                continue
            axis_target = safe.copy()
            axis_target[axis] = raw[axis]
            attempts.append((axis_target, 1.0, 0.0))
            attempts.append((axis_target, 0.5, 0.0))
        reason = "no safe candidate"
        accepted = False
        accepted_orientation_factor = 0.0
        for desired, position_factor, orientation_factor in attempts:
            accepted, reason = self._attempt_candidate(
                side, desired, position_factor, orientation_factor
            )
            if accepted:
                accepted_orientation_factor = orientation_factor
                break

        # A safe position-only fallback must not starve wrist orientation.
        # Continue with one orientation-only safety-checked step from the
        # posture accepted above. If the combined candidate already included
        # orientation, do not apply a second rotation in the same frame.
        if not accepted or accepted_orientation_factor == 0.0:
            orientation_start = self.safe_targets[side].copy()
            if quat_error_deg(orientation_start[3:], raw[3:]) > 0.1:
                orientation_target = orientation_start.copy()
                orientation_target[3:] = approach_quat(
                    orientation_start[3:],
                    raw[3:],
                    self.target_orientation_step,
                )
                orientation_accepted, orientation_reason = self._attempt_candidate(
                    side, orientation_target, 0.0, 1.0
                )
                if orientation_accepted:
                    accepted = True
                elif not accepted:
                    reason = orientation_reason
        self.projected[side] = (
            not accepted
            or np.linalg.norm(self.safe_targets[side][:3] - raw[:3]) > 0.01
            or quat_error_deg(self.safe_targets[side][3:], raw[3:]) > 3.0
        )
        self.last_projection_reason[side] = "" if accepted else reason
        if not accepted:
            self._set_mocap(side, self.safe_targets[side])

    def _draw_raw_targets(self, viewer: object) -> None:
        scene = viewer.user_scn
        scene.ngeom = 0
        for side, color in (
            ("left", np.array([0.2, 0.65, 1.0, 0.35])),
            ("right", np.array([1.0, 0.65, 0.1, 0.35])),
        ):
            raw = self.raw_targets[side]
            safe = self.safe_targets[side]
            # Dim axes: raw hand-mapped pose. Bright axes: pose currently
            # accepted by IK and collision projection.
            add_target_frame(
                scene, raw, reachable=True, scale=0.065, active=False
            )
            add_target_frame(
                scene, safe, reachable=True, scale=0.045, active=True
            )
            if scene.ngeom < scene.maxgeom:
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(
                    geom,
                    mujoco.mjtGeom.mjGEOM_CAPSULE,
                    np.ones(3),
                    np.zeros(3),
                    np.eye(3).ravel(),
                    color.astype(np.float32),
                )
                mujoco.mjv_connector(
                    geom,
                    mujoco.mjtGeom.mjGEOM_CAPSULE,
                    0.003,
                    safe[:3],
                    raw[:3],
                )
                scene.ngeom += 1

    def run(self) -> None:
        calibration_summary = (
            "facing/down/T-pose"
            if self.mapping_mode == "shared"
            else "facing/down/T-pose/elbow-90"
        )
        print(
            f"ABSOLUTE VIVE SIMULATION: mode={self.mapping_mode}; K starts "
            f"{calibration_summary} calibration, TRIGGER captures each pose "
            "(C is backup); "
            "grips are permissions; touchpad controls grippers; "
            "H=safe home; R=reset selected target; Esc=quit.",
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
                        order = (
                            ("right", "left")
                            if self.solve_order_flipped
                            else ("left", "right")
                        )
                        self.solve_order_flipped = not self.solve_order_flipped
                        displayed_side = self.side
                        if self.home_reset_active:
                            self._step_home_reset()
                        else:
                            for side in order:
                                self.solve_projected_side(side)
                        self._apply_gripper_positions()
                        self.after_solve()
                        self.side = displayed_side
                        self.target = self.safe_targets[self.side].copy()
                        self.ik_target = self.vive_ik_targets[self.side].copy()
                        self._sync_control_sliders()
                        self._draw_raw_targets(viewer)
                    if self.close_requested:
                        viewer.close()
                        continue
                    viewer.sync()
                    time.sleep(
                        max(0.0, 1.0 / 60.0 - (time.monotonic() - start))
                    )
        finally:
            self._close_headset_guidance()
            openvr.shutdown()
            self._close_calibration_ui()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scale-forward", type=float, default=1.0)
    parser.add_argument("--scale-left", type=float, default=1.0)
    parser.add_argument("--scale-up", type=float, default=1.0)
    parser.add_argument("--position-step-mm", type=float, default=6.0)
    parser.add_argument("--orientation-step-deg", type=float, default=2.5)
    parser.add_argument("--gripper-step-mm", type=float, default=1.0)
    parser.add_argument("--shoulder-width", type=float, default=0.38)
    parser.add_argument("--shoulder-drop", type=float, default=0.27)
    parser.add_argument(
        "--mapping-mode",
        choices=("anchored", "shared"),
        default="anchored",
        help="anchored=independent arm anchors; shared=one absolute workspace",
    )
    args = parser.parse_args()
    scales = np.array(
        [args.scale_forward, args.scale_left, args.scale_up], dtype=np.float64
    )
    if np.any(scales < 0.2) or np.any(scales > 1.5):
        parser.error("body mapping scales must be between 0.2 and 1.5")
    if not 0.25 <= args.shoulder_width <= 0.60:
        parser.error("--shoulder-width must be between 0.25 and 0.60 m")
    if not 0.18 <= args.shoulder_drop <= 0.40:
        parser.error("--shoulder-drop must be between 0.18 and 0.40 m")
    ViveAbsoluteSimulationApp(
        scale_xyz=scales,
        shoulder_width=args.shoulder_width,
        shoulder_drop=args.shoulder_drop,
        mapping_mode=args.mapping_mode,
        side="right",
        scale=1.0,
        max_offset=0.5,
        elbow_bend_deg=45.0,
        position_step_mm=args.position_step_mm,
        orientation_step_deg=args.orientation_step_deg,
        gripper_step_mm=args.gripper_step_mm,
    ).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
