#!/usr/bin/env python3
"""Guarded VIVE teleoperation for the physical OpenArm 1.0 bimanual robot."""

from __future__ import annotations

import argparse
import pathlib
import time

import glfw
import mujoco
import numpy as np

from interactive_mujoco_ee_drag import (
    limit_joint_command,
    physical_to_model_position,
)
from vive_mujoco_teleop import ViveSimulationApp


class BimanualHardwareBridge:
    """Explicitly enabled, rate-limited two-arm physical driver bridge."""

    def __init__(
        self,
        config_path: pathlib.Path,
        command_hz: float,
        max_step: float,
        max_tracking_error: float,
        joint_limit_margin: float = 0.05,
        max_gripper_step: float | None = None,
        max_joint_acceleration: float = 1.5,
    ) -> None:
        from openarm_driver import Config, SingleArmDriver

        config = Config(config_path)
        self.drivers = {
            side: SingleArmDriver(f"{side}_arm", config)
            for side in ("left", "right")
        }
        self.joint_limits = {
            side: config.get_joint_limits(f"{side}_arm") for side in self.drivers
        }
        self.command_limits = {
            side: limits.copy() for side, limits in self.joint_limits.items()
        }
        for limits in self.command_limits.values():
            limits[:7, 0] += joint_limit_margin
            limits[:7, 1] -= joint_limit_margin
            if np.any(limits[:7, 0] >= limits[:7, 1]):
                raise ValueError("joint-limit margin leaves an empty arm range")
        self.joint_limit_margin = float(joint_limit_margin)
        self.command_period = 1.0 / command_hz
        self.max_step = max_step
        self.max_joint_acceleration = float(max_joint_acceleration)
        self.max_step_acceleration = (
            self.max_joint_acceleration * self.command_period**2
        )
        self.max_gripper_step = (
            max_step if max_gripper_step is None else float(max_gripper_step)
        )
        self.max_tracking_error = max_tracking_error
        self.last_command_time = {"left": 0.0, "right": 0.0}
        self.last_command_delta = {
            "left": np.zeros(8, dtype=np.float64),
            "right": np.zeros(8, dtype=np.float64),
        }
        self.armed = False
        try:
            positions = self.read_positions_now(stable=True)
            for side, position in positions.items():
                self._check_limits(side, position)
                self.drivers[side].last_command = position.copy()
        finally:
            for driver in self.drivers.values():
                driver.openarm.disable_all()

    def _check_limits(self, side: str, position: np.ndarray) -> None:
        limits = self.joint_limits[side]
        if np.any(position < limits[:, 0]) or np.any(position > limits[:, 1]):
            bad = np.flatnonzero(
                (position < limits[:, 0]) | (position > limits[:, 1])
            ).tolist()
            raise RuntimeError(f"{side} joint limits violated: {bad}")

    def read_positions_now(self, stable: bool = False) -> dict[str, np.ndarray]:
        count = 5 if stable else 1
        samples: dict[str, list[np.ndarray]] = {"left": [], "right": []}
        for _ in range(count):
            for side, driver in self.drivers.items():
                value = np.asarray(
                    driver.fetch_position(refresh=True), dtype=np.float64
                )
                if value.shape != (8,) or not np.all(np.isfinite(value)):
                    raise RuntimeError(f"invalid {side} physical feedback")
                samples[side].append(value)
            if stable:
                time.sleep(0.02)
        if stable:
            for side in samples:
                if float(np.max(np.ptp(samples[side], axis=0))) > 0.02:
                    raise RuntimeError(f"unstable {side} physical feedback")
        return {side: values[-1].copy() for side, values in samples.items()}

    def arm_both(self) -> None:
        if self.armed:
            return
        positions = self.read_positions_now(stable=True)
        started: list[str] = []
        try:
            for side in ("left", "right"):
                self._check_limits(side, positions[side])
                self.drivers[side].last_command = positions[side].copy()
                self.drivers[side].start()
                started.append(side)
            self.armed = True
            self.last_command_time = {"left": 0.0, "right": 0.0}
            for side in self.last_command_delta:
                self.last_command_delta[side].fill(0.0)
            print("[hardware] BOTH ARMS ENABLED at measured posture", flush=True)
        except Exception:
            for side in started:
                self.drivers[side].stop()
            raise

    def disarm(self) -> None:
        was_armed = self.armed
        self.armed = False
        for driver in self.drivers.values():
            try:
                driver.stop()
            except Exception:
                driver.openarm.disable_all()
        if was_armed:
            print("[hardware] BOTH ARMS DISABLED", flush=True)

    def command_due(self, side: str) -> bool:
        return (
            time.monotonic() - self.last_command_time[side] >= self.command_period
        )

    def clamp_to_command_limits(
        self, side: str, desired: np.ndarray
    ) -> np.ndarray:
        """Project a command inside the soft range before the hard checker."""

        desired = np.asarray(desired, dtype=np.float64)
        limits = self.command_limits[side]
        return np.clip(desired, limits[:, 0], limits[:, 1])

    def limit_command(
        self, side: str, previous: np.ndarray, desired: np.ndarray
    ) -> np.ndarray:
        """Apply velocity and acceleration limits to the next joint command."""

        per_joint_step = np.full(8, self.max_step, dtype=np.float64)
        per_joint_step[7] = self.max_gripper_step
        velocity_limited = limit_joint_command(previous, desired, per_joint_step)
        requested_delta = velocity_limited - previous
        prior_delta = self.last_command_delta[side]
        arm_delta = np.clip(
            requested_delta[:7],
            prior_delta[:7] - self.max_step_acceleration,
            prior_delta[:7] + self.max_step_acceleration,
        )
        reversing = arm_delta * requested_delta[:7] < 0.0
        arm_delta[reversing] = 0.0
        overshooting = np.abs(arm_delta) > np.abs(requested_delta[:7])
        arm_delta[overshooting] = requested_delta[:7][overshooting]
        result = previous.copy()
        result[:7] += arm_delta
        result[7] += requested_delta[7]
        return result

    def send_if_due(self, side: str, desired: np.ndarray) -> np.ndarray | None:
        if not self.armed or not self.command_due(side):
            return None
        driver = self.drivers[side]
        desired = self.clamp_to_command_limits(side, desired)
        self._check_limits(side, desired)
        previous = driver.last_command.copy()
        limited = self.limit_command(side, previous, desired)
        driver.send_position(limited)
        measured = np.asarray(driver.fetch_position(refresh=True), dtype=np.float64)
        if measured.shape != (8,) or not np.all(np.isfinite(measured)):
            raise RuntimeError(f"lost {side} feedback")
        tracking_error = np.abs(measured - limited)
        if float(np.max(tracking_error)) > self.max_tracking_error:
            index = int(np.argmax(tracking_error))
            joint = f"J{index + 1}" if index < 7 else "J8/gripper"
            raise RuntimeError(
                f"{side} {joint} tracking error "
                f"{tracking_error[index]:.4f} rad exceeded "
                f"{self.max_tracking_error:.4f}; "
                f"command={limited[index]:.4f}, measured={measured[index]:.4f}"
            )
        driver.last_command = limited.copy()
        self.last_command_delta[side] = limited - previous
        self.last_command_time[side] = time.monotonic()
        return measured

    def close(self) -> None:
        self.disarm()
        for driver in self.drivers.values():
            driver.openarm.disable_all()


class VivePhysicalApp(ViveSimulationApp):
    def __init__(self, bridge: BimanualHardwareBridge, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self.hardware = bridge
        self._sync_to_physical()

    @staticmethod
    def _physical_gripper(side: str, model_position: float) -> float:
        direction = -1.0 if side == "right" else 1.0
        return direction * model_position * (np.pi / 3.0) / 0.044

    def _sync_to_physical(self) -> None:
        self.hardware.disarm()
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
            self.gripper_position[side] = float(
                np.mean([self.data.qpos[q] for q in self.finger_qpos[side]])
            )
        print("[hardware] simulation synchronized; motors remain disabled", flush=True)

    def _handle_native_keys(self, viewer: object) -> None:
        forwarded: list[int] = []
        while self.pending_keys:
            key = self.pending_keys.pop(0)
            if key in (ord("E"), ord("e")):
                if self.hardware.armed:
                    self.hardware.disarm()
                else:
                    self._sync_to_physical()
                    self.hardware.arm_both()
            elif key in (ord("P"), ord("p")):
                self._sync_to_physical()
            elif key in (ord("F"), ord("f"), ord("D"), ord("d")):
                print("[hardware] VIVE mode uses E + controller inputs", flush=True)
            else:
                forwarded.append(key)
        self.pending_keys.extend(forwarded)
        super()._handle_native_keys(viewer)

    def after_solve(self) -> None:
        if not self.hardware.armed:
            return
        lost_while_held = [
            side
            for side in ("left", "right")
            if self.controller_grip_pressed[side] and not self.tracking_valid[side]
        ]
        if lost_while_held:
            self.hardware.disarm()
            print(
                f"[hardware] SAFETY STOP: controller tracking lost while held: "
                f"{lost_while_held}",
                flush=True,
            )
            return
        right, left = self._drivers()
        desired_by_side = {"right": right.copy(), "left": left.copy()}
        physical = {
            side: driver.last_command.copy()
            for side, driver in self.hardware.drivers.items()
        }
        try:
            for side in ("left", "right"):
                if not (
                    self.grip_active[side] or self.gripper_action[side] != "HOLD"
                ):
                    continue
                desired = desired_by_side[side]
                desired[7] = self._physical_gripper(
                    side, self.gripper_position[side]
                )
                if not self.hardware.command_due(side):
                    continue
                next_command = self.hardware.limit_command(
                    side, physical[side], desired
                )
                self._validate_physical_path(
                    side, next_command, physical=physical, verbose=False
                )
                self.hardware.send_if_due(side, desired)
                physical[side] = next_command
        except Exception as error:
            self.hardware.disarm()
            print(f"[hardware] SAFETY STOP: {error}", flush=True)

    def run(self) -> None:
        print(
            "REAL VIVE MODE: motors start disabled. E=enable/disable both, "
            "P=resync+disable. Keep E-stop accessible.",
            flush=True,
        )
        try:
            super().run()
        finally:
            self.hardware.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--confirm-hardware",
        action="store_true",
        help="confirm clear workspace and accessible emergency stop",
    )
    parser.add_argument(
        "--config",
        type=pathlib.Path,
        default=pathlib.Path("openarm_safe_current.yaml"),
    )
    parser.add_argument("--command-hz", type=float, default=20.0)
    parser.add_argument("--max-joint-step", type=float, default=0.002)
    parser.add_argument("--max-tracking-error", type=float, default=0.15)
    parser.add_argument("--scale", type=float, default=0.6)
    parser.add_argument("--max-offset", type=float, default=0.15)
    args = parser.parse_args()
    if not args.confirm_hardware:
        parser.error("physical mode requires --confirm-hardware")
    if not 5.0 <= args.command_hz <= 50.0:
        parser.error("--command-hz must be between 5 and 50")
    if not 0.0005 <= args.max_joint_step <= 0.005:
        parser.error("--max-joint-step must be between 0.0005 and 0.005 rad")
    if not 0.05 <= args.max_tracking_error <= 0.3:
        parser.error("--max-tracking-error must be between 0.05 and 0.3 rad")
    bridge = BimanualHardwareBridge(
        args.config,
        args.command_hz,
        args.max_joint_step,
        args.max_tracking_error,
    )
    VivePhysicalApp(
        bridge=bridge,
        side="right",
        scale=args.scale,
        max_offset=args.max_offset,
        elbow_bend_deg=45.0,
        position_step_mm=4.0,
        orientation_step_deg=0.75,
        gripper_step_mm=0.4,
    ).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
