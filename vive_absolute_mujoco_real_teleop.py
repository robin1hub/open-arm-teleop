#!/usr/bin/env python3
"""Guarded absolute-body-frame VIVE teleoperation for physical OpenArm 1.0."""

from __future__ import annotations

import argparse
import pathlib
import time

import mujoco
import numpy as np

from interactive_mujoco_ee_drag import (
    physical_to_model_position,
)
from vive_absolute_mujoco_teleop import ViveAbsoluteSimulationApp
from vive_mujoco_real_teleop import BimanualHardwareBridge


class ViveAbsolutePhysicalApp(ViveAbsoluteSimulationApp):
    """Absolute VIVE mapping plus an explicitly armed physical output layer."""

    def __init__(
        self,
        bridge: BimanualHardwareBridge,
        tracking_loss_timeout: float,
        **kwargs: object,
    ) -> None:
        super().__init__(**kwargs)
        self.hardware = bridge
        self.tracking_loss_timeout = float(tracking_loss_timeout)
        self.tracking_lost_since: dict[str, float | None] = {
            "left": None,
            "right": None,
        }
        self.hardware_home_active = False
        self._sync_to_physical()

    @staticmethod
    def _physical_gripper(side: str, model_position: float) -> float:
        direction = -1.0 if side == "right" else 1.0
        return direction * model_position * (np.pi / 3.0) / 0.044

    def _invalidate_calibration(self) -> None:
        self.calibrated = False
        self.calibration_stage = 0
        self.calibration_session_active = False
        self.grip_active = {"left": False, "right": False}
        print(
            "[absolute] calibration cleared; release grips and press K to start",
            flush=True,
        )

    def _sync_to_physical(self) -> None:
        self.hardware.disarm()
        self.hardware_home_active = False
        self.home_reset_active = False
        positions = self.hardware.read_positions_now(stable=True)
        for side, physical in positions.items():
            self.hardware.drivers[side].last_command = physical.copy()
            model_position = physical_to_model_position(side, physical, "v1")
            self.resolver.set_qpos(self.data.qpos, model_position, side)
        mujoco.mj_forward(self.model, self.data)
        self._sync_from_model()
        for side in ("left", "right"):
            pose = self._fk(side)
            self._set_mocap(side, pose)
            self.vive_ik_targets[side] = pose.copy()
            self.safe_targets[side] = pose.copy()
            self.raw_targets[side] = pose.copy()
            self.gripper_position[side] = float(
                np.mean([self.data.qpos[q] for q in self.finger_qpos[side]])
            )
        self._invalidate_calibration()
        print("[hardware] simulation synchronized; motors remain disabled", flush=True)

    def _sync_model_to_commanded_posture(self) -> None:
        """Render the exact rate-limited posture most recently sent."""

        for side, driver in self.hardware.drivers.items():
            model_position = physical_to_model_position(
                side, driver.last_command, "v1"
            )
            self.resolver.set_qpos(self.data.qpos, model_position, side)
        mujoco.mj_forward(self.model, self.data)
        self._sync_from_model()
        # The next IK step must start from the same posture that is rendered
        # and sent to hardware. Leaving these targets ahead of qpos makes the
        # IK discontinuity guard reject every later frame as a large jump.
        for side in ("left", "right"):
            pose = self._fk(side)
            self.vive_ik_targets[side] = pose.copy()
            self.safe_targets[side] = pose.copy()
            self._set_mocap(side, pose)

    def _handle_native_keys(self, viewer: object) -> None:
        forwarded: list[int] = []
        while self.pending_keys:
            key = self.pending_keys.pop(0)
            if key in (ord("E"), ord("e")):
                if self.hardware.armed:
                    self.hardware_home_active = False
                    self.home_reset_active = False
                    self.hardware.disarm()
                elif not self.calibrated:
                    print("[hardware] E ignored: calibrate with C first", flush=True)
                elif any(self.grip_active.values()):
                    print("[hardware] E ignored: release both grips first", flush=True)
                else:
                    try:
                        self.hardware.arm_both()
                    except Exception as error:
                        self.hardware.disarm()
                        print(
                            f"[hardware] ENABLE rejected; motors remain "
                            f"disabled: {error}",
                            flush=True,
                        )
            elif key in (ord("P"), ord("p")):
                self._sync_to_physical()
            elif key in (ord("K"), ord("k")):
                self.hardware_home_active = False
                self.home_reset_active = False
                self.hardware.disarm()
                forwarded.append(key)
            elif key in (ord("H"), ord("h")):
                if not self.hardware.armed:
                    print(
                        "[hardware] H ignored: motors are disabled; "
                        "press P to synchronize or E to enable first",
                        flush=True,
                    )
                else:
                    self._begin_home_reset()
                    self.hardware_home_active = True
                    print(
                        "[hardware] REAL HOME active; E cancels and disables",
                        flush=True,
                    )
            elif key in (ord("F"), ord("f"), ord("D"), ord("d")):
                print("[hardware] absolute mode uses C, E and controller grips", flush=True)
            else:
                forwarded.append(key)
        self.pending_keys.extend(forwarded)
        super()._handle_native_keys(viewer)

    def update_vive_target(self) -> None:
        super().update_vive_target()
        if not self.hardware_home_active:
            return
        # Physical HOME owns both arms until the rate-limited CAN commands
        # have actually converged, even if the faster simulation is already
        # at its home pose.
        for side in ("left", "right"):
            self.grip_active[side] = False
            self.require_grip_release[side] = True

    def _attempt_candidate(
        self,
        side: str,
        desired: np.ndarray,
        position_factor: float,
        orientation_factor: float,
    ) -> tuple[bool, str]:
        """Reject IK candidates before they enter the physical soft-limit band."""

        qpos_before = self.data.qpos.copy()
        ik_before = self.vive_ik_targets[side].copy()
        safe_before = self.safe_targets[side].copy()
        accepted, reason = super()._attempt_candidate(
            side, desired, position_factor, orientation_factor
        )
        if not accepted:
            return accepted, reason

        right, left = self._drivers()
        candidate = right if side == "right" else left
        limits = self.hardware.command_limits[side]
        outside = (candidate[:7] < limits[:7, 0]) | (
            candidate[:7] > limits[:7, 1]
        )
        if not np.any(outside):
            return True, ""

        self._restore_candidate(qpos_before, ik_before, side)
        self.safe_targets[side] = safe_before
        self._set_mocap(side, safe_before)
        joints = (np.flatnonzero(outside) + 1).tolist()
        return False, f"physical soft joint limit: J{joints}"

    def after_solve(self) -> None:
        if not self.hardware.armed:
            return
        now = time.monotonic()
        lost_while_held = [
            side
            for side in ("left", "right")
            if self.controller_grip_pressed[side] and not self.tracking_valid[side]
        ]
        for side in ("left", "right"):
            if side in lost_while_held:
                if self.tracking_lost_since[side] is None:
                    self.tracking_lost_since[side] = now
                    print(
                        f"[hardware] {side} tracking temporarily lost while "
                        f"held; output frozen for up to "
                        f"{self.tracking_loss_timeout:.2f}s",
                        flush=True,
                    )
            else:
                self.tracking_lost_since[side] = None
        sustained_loss = [
            side
            for side in lost_while_held
            if self.tracking_lost_since[side] is not None
            and now - self.tracking_lost_since[side]
            >= self.tracking_loss_timeout
        ]
        if sustained_loss:
            self.hardware_home_active = False
            self.hardware.disarm()
            print(
                f"[hardware] SAFETY STOP: tracking lost for at least "
                f"{self.tracking_loss_timeout:.2f}s while held: "
                f"{sustained_loss}",
                flush=True,
            )
            return

        right, left = self._drivers()
        desired_by_side = {"right": right.copy(), "left": left.copy()}
        physical = {
            side: driver.last_command.copy()
            for side, driver in self.hardware.drivers.items()
        }
        home_feedback_sides: set[str] = set()
        try:
            for side in ("left", "right"):
                if not (
                    self.hardware_home_active
                    or self.grip_active[side]
                    or self.gripper_action[side] != "HOLD"
                ):
                    continue
                desired = desired_by_side[side]
                desired[7] = self._physical_gripper(
                    side, self.gripper_position[side]
                )
                desired = self.hardware.clamp_to_command_limits(side, desired)
                if not self.hardware.command_due(side):
                    continue
                next_command = self.hardware.limit_command(
                    side, physical[side], desired
                )
                self._validate_physical_path(
                    side, next_command, physical=physical, verbose=False
                )
                measured = self.hardware.send_if_due(side, desired)
                if measured is not None:
                    physical[side] = self.hardware.drivers[
                        side
                    ].last_command.copy()
                    if self.hardware_home_active:
                        home_feedback_sides.add(side)
            if self.hardware_home_active:
                home_error = max(
                    float(
                        np.max(
                            np.abs(
                                desired_by_side[side][:7]
                                - physical[side][:7]
                            )
                        )
                    )
                    for side in ("left", "right")
                )
                if (
                    not self.home_reset_active
                    and home_feedback_sides == {"left", "right"}
                    and home_error <= max(0.01, 2.0 * self.hardware.max_step)
                ):
                    self.hardware_home_active = False
                    print(
                        "[hardware] REAL HOME complete; release both grips "
                        "before resuming control",
                        flush=True,
                    )
            # Keep the visible robot on the same trajectory as the command
            # sent over CAN. Raw/safe target frames still show requested pose.
            self._sync_model_to_commanded_posture()
        except Exception as error:
            self.hardware_home_active = False
            self.home_reset_active = False
            self.hardware.disarm()
            print(f"[hardware] SAFETY STOP: {error}", flush=True)

    def run(self) -> None:
        print(
            "REAL ABSOLUTE VIVE MODE: P=resync+disable, K=start calibration, "
            "TRIGGER/C=capture pose, "
            "E=enable/disable, H=rate-limited safe home. "
            "Keep the emergency stop accessible.",
            flush=True,
        )
        try:
            super().run()
        finally:
            self.hardware.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm-hardware", action="store_true")
    parser.add_argument(
        "--config",
        type=pathlib.Path,
        default=pathlib.Path("openarm_safe_raw_zero.yaml"),
    )
    parser.add_argument("--command-hz", type=float, default=20.0)
    parser.add_argument("--max-joint-step", type=float, default=0.002)
    parser.add_argument("--max-joint-acceleration", type=float, default=1.5)
    parser.add_argument("--max-gripper-step", type=float, default=0.001)
    parser.add_argument("--max-tracking-error", type=float, default=0.15)
    parser.add_argument("--joint-limit-margin", type=float, default=0.05)
    parser.add_argument("--tracking-loss-timeout", type=float, default=0.30)
    parser.add_argument("--scale-forward", type=float, default=1.0)
    parser.add_argument("--scale-left", type=float, default=1.0)
    parser.add_argument("--scale-up", type=float, default=1.0)
    parser.add_argument("--position-step-mm", type=float, default=4.0)
    parser.add_argument("--orientation-step-deg", type=float, default=0.75)
    parser.add_argument("--gripper-step-mm", type=float, default=0.4)
    parser.add_argument("--shoulder-width", type=float, default=0.38)
    parser.add_argument("--shoulder-drop", type=float, default=0.27)
    parser.add_argument(
        "--mapping-mode", choices=("anchored", "shared"), default="anchored"
    )
    args = parser.parse_args()
    if not args.confirm_hardware:
        parser.error("physical mode requires --confirm-hardware")
    if not 5.0 <= args.command_hz <= 50.0:
        parser.error("--command-hz must be between 5 and 50")
    if not 0.0005 <= args.max_joint_step <= 0.015:
        parser.error("--max-joint-step must be between 0.0005 and 0.015 rad")
    if not 0.2 <= args.max_joint_acceleration <= 3.0:
        parser.error(
            "--max-joint-acceleration must be between 0.2 and 3.0 rad/s^2"
        )
    if not 0.0005 <= args.max_gripper_step <= 0.003:
        parser.error("--max-gripper-step must be between 0.0005 and 0.003 rad")
    if not 0.05 <= args.max_tracking_error <= 0.3:
        parser.error("--max-tracking-error must be between 0.05 and 0.3 rad")
    if not 0.02 <= args.joint_limit_margin <= 0.15:
        parser.error("--joint-limit-margin must be between 0.02 and 0.15 rad")
    if not 0.10 <= args.tracking_loss_timeout <= 0.75:
        parser.error("--tracking-loss-timeout must be between 0.10 and 0.75 s")
    scales = np.array(
        [args.scale_forward, args.scale_left, args.scale_up], dtype=np.float64
    )
    if np.any(scales < 0.2) or np.any(scales > 1.0):
        parser.error("physical body mapping scales must be between 0.2 and 1.0")
    if not 0.25 <= args.shoulder_width <= 0.60:
        parser.error("--shoulder-width must be between 0.25 and 0.60 m")
    if not 0.18 <= args.shoulder_drop <= 0.40:
        parser.error("--shoulder-drop must be between 0.18 and 0.40 m")

    bridge = BimanualHardwareBridge(
        args.config,
        args.command_hz,
        args.max_joint_step,
        args.max_tracking_error,
        args.joint_limit_margin,
        args.max_gripper_step,
        args.max_joint_acceleration,
    )
    ViveAbsolutePhysicalApp(
        bridge=bridge,
        tracking_loss_timeout=args.tracking_loss_timeout,
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
