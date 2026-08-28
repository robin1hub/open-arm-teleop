#!/usr/bin/env python3
"""Official-driver VIVE teleoperation for physical OpenArm 1.0."""

from __future__ import annotations

import argparse
from collections import deque
import pathlib
import time

import mujoco
import numpy as np

from .interactive_mujoco_ee_drag import (
    physical_gripper_direction,
    physical_to_model_position,
)
from .vive_mujoco_teleop import ViveSimulationApp


class BimanualHardwareBridge:
    """Explicitly enabled bridge using only openarm-driver safety checks."""

    def __init__(
        self,
        config_path: pathlib.Path,
        command_hz: float,
    ) -> None:
        from openarm_driver import Config, SingleArmDriver

        config = Config(config_path)
        if not hasattr(config, "get_joint_velocity_limits"):
            raise RuntimeError(
                "physical VIVE mode requires openarm-driver>=0.3.0"
            )
        velocity_limits = config.get_joint_velocity_limits()
        if velocity_limits is None:
            raise RuntimeError(
                "physical config must define official joint_velocity_limits"
            )
        self.official_velocity_limits = np.asarray(
            velocity_limits, dtype=np.float64
        )
        self.drivers = {
            side: SingleArmDriver(f"{side}_arm", config)
            for side in ("left", "right")
        }
        self.command_hz = float(command_hz)
        self.command_period = 1.0 / self.command_hz
        if self.official_velocity_limits.shape != (8,):
            raise RuntimeError("official joint_velocity_limits must contain J1-J8")
        self.last_command_time = {"left": 0.0, "right": 0.0}
        self.next_command_time = {"left": 0.0, "right": 0.0}
        self.command_intervals = {
            "left": deque(maxlen=120),
            "right": deque(maxlen=120),
        }
        self.last_cadence_log = time.monotonic()
        self.armed = False
        try:
            positions = self.read_positions_now(stable=True)
            for side, position in positions.items():
                self.drivers[side].last_command = position.copy()
        finally:
            for driver in self.drivers.values():
                driver.openarm.disable_all()

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
                self.drivers[side].last_command = positions[side].copy()
                self.drivers[side].start()
                started.append(side)
            self.armed = True
            self.last_command_time = {"left": 0.0, "right": 0.0}
            self.next_command_time = {"left": 0.0, "right": 0.0}
            for side in self.command_intervals:
                self.command_intervals[side].clear()
            self.last_cadence_log = time.monotonic()
            limits = ",".join(
                f"{value:.2f}" for value in self.official_velocity_limits
            )
            print(
                "[hardware] BOTH ARMS ENABLED at measured posture; "
                f"official J1-J8 velocity limits=[{limits}] rad/s",
                flush=True,
            )
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

    def command_due(self, side: str, now: float | None = None) -> bool:
        """Return whether the phase-locked command deadline has arrived."""

        current = time.monotonic() if now is None else float(now)
        return current >= self.next_command_time[side]

    def _mark_command_sent(self, side: str, sent_at: float) -> None:
        """Advance from the previous deadline instead of quantizing to UI frames."""

        previous_time = self.last_command_time[side]
        if previous_time > 0.0:
            self.command_intervals[side].append(sent_at - previous_time)
        self.last_command_time[side] = sent_at

        deadline = self.next_command_time[side]
        if deadline <= 0.0:
            self.next_command_time[side] = sent_at + self.command_period
            return
        missed_periods = max(
            1, int(np.floor((sent_at - deadline) / self.command_period)) + 1
        )
        self.next_command_time[side] = deadline + missed_periods * self.command_period

    def _maybe_log_cadence(self, now: float) -> None:
        elapsed = now - self.last_cadence_log
        if elapsed < 1.0:
            return
        details = []
        for side in ("left", "right"):
            intervals = self.command_intervals[side]
            active = (
                self.last_command_time[side] > 0.0
                and now - self.last_command_time[side]
                <= 2.0 * self.command_period
            )
            actual_hz = (
                1.0 / float(np.mean(intervals)) if active and intervals else 0.0
            )
            details.append(f"{side}={actual_hz:.1f}Hz")
        print(
            f"[hardware] cadence target={self.command_hz:.1f}Hz "
            + " ".join(details),
            flush=True,
        )
        self.last_cadence_log = now

    def send_if_due(
        self,
        side: str,
        desired: np.ndarray,
    ) -> np.ndarray | None:
        now = time.monotonic()
        if not self.armed or not self.command_due(side, now):
            return None
        driver = self.drivers[side]
        driver.send_position(desired)
        # SingleArmDriver.send_position() already receives a fresh motor state.
        # A second refresh here doubled CAN transactions and made the UI-owned
        # command cadence uneven.
        latest_state = driver.latest_state
        if not isinstance(latest_state, dict) or "qpos" not in latest_state:
            raise RuntimeError(f"lost {side} feedback after position command")
        measured = np.asarray(latest_state["qpos"], dtype=np.float64)
        if measured.shape != (8,) or not np.all(np.isfinite(measured)):
            raise RuntimeError(f"lost {side} feedback")
        sent_at = time.monotonic()
        self._mark_command_sent(side, sent_at)
        self._maybe_log_cadence(sent_at)
        return measured

    def close(self) -> None:
        self.disarm()
        for driver in self.drivers.values():
            driver.openarm.disable_all()


class VivePhysicalApp(ViveSimulationApp):
    def __init__(self, bridge: BimanualHardwareBridge, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self.hardware = bridge
        self.target_position_step = np.inf
        self.target_orientation_step = np.pi
        self.max_ik_joint_step = np.inf
        self.kin.set_max_joint_change_per_solve(np.inf)
        self.gripper_step = 0.044
        self._sync_to_physical()

    @staticmethod
    def _physical_gripper(side: str, model_position: float) -> float:
        direction = physical_gripper_direction(side)
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
                self.hardware.send_if_due(side, desired)
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
        default=pathlib.Path(__file__).resolve().parents[2]
        / "config/openarm_safe_current.yaml",
    )
    parser.add_argument("--command-hz", type=float, default=60.0)
    parser.add_argument("--scale", type=float, default=0.6)
    parser.add_argument("--max-offset", type=float, default=0.15)
    args = parser.parse_args()
    if not args.confirm_hardware:
        parser.error("physical mode requires --confirm-hardware")
    if not 5.0 <= args.command_hz <= 100.0:
        parser.error("--command-hz must be between 5 and 100")
    bridge = BimanualHardwareBridge(
        config_path=args.config,
        command_hz=args.command_hz,
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
