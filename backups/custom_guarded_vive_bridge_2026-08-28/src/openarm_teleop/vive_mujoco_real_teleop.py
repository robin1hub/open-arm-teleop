#!/usr/bin/env python3
"""Guarded VIVE teleoperation for the physical OpenArm 1.0 bimanual robot."""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable
import pathlib
import time

import glfw
import mujoco
import numpy as np

from .interactive_mujoco_ee_drag import physical_to_model_position
from .vive_mujoco_teleop import ViveSimulationApp


# OpenArm's current upstream openarm_cell profile. The physical launcher starts
# at 50% of these limits and retains a much smaller single-command jump guard.
UPSTREAM_ARM_VELOCITY_LIMITS = np.array(
    [2.0, 2.0, 3.3, 3.3, 6.3, 6.3, 6.3], dtype=np.float64
)
MAX_COMMAND_DT_S = 0.1


def add_hardware_motion_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the shared, time-based physical motion limit arguments."""

    parser.add_argument("--command-hz", type=float, default=60.0)
    parser.add_argument(
        "--joint-velocity-scale",
        type=float,
        default=0.5,
        help="fraction of the upstream OpenArm J1-J7 velocity profile",
    )
    parser.add_argument(
        "--joint-velocity-limits",
        type=float,
        nargs=7,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="explicit J1-J7 limits in rad/s; overrides --joint-velocity-scale",
    )
    parser.add_argument(
        "--max-command-step",
        "--max-joint-step",
        dest="max_command_step",
        type=float,
        default=0.028,
        help="single-command jump guard in rad (must stay below 0.03)",
    )
    parser.add_argument("--max-joint-acceleration", type=float, default=6.0)
    parser.add_argument("--gripper-velocity-limit", type=float, default=0.04)
    parser.add_argument(
        "--max-gripper-step",
        type=float,
        default=None,
        help="deprecated per-command J8 limit; converted using --command-hz",
    )
    parser.add_argument("--max-tracking-error", type=float, default=0.15)


def resolve_hardware_motion_arguments(
    args: argparse.Namespace, parser: argparse.ArgumentParser
) -> tuple[np.ndarray, float]:
    """Validate CLI limits and return J1-J7 and J8 velocities in rad/s."""

    if not 5.0 <= args.command_hz <= 100.0:
        parser.error("--command-hz must be between 5 and 100")
    if not 0.1 <= args.joint_velocity_scale <= 1.0:
        parser.error("--joint-velocity-scale must be between 0.1 and 1.0")
    if args.joint_velocity_limits is None:
        velocity_limits = (
            UPSTREAM_ARM_VELOCITY_LIMITS * args.joint_velocity_scale
        )
    else:
        velocity_limits = np.asarray(
            args.joint_velocity_limits, dtype=np.float64
        )
        if (
            not np.all(np.isfinite(velocity_limits))
            or np.any(velocity_limits <= 0.0)
            or np.any(velocity_limits > UPSTREAM_ARM_VELOCITY_LIMITS)
        ):
            parser.error(
                "--joint-velocity-limits values must be positive and no higher "
                "than the upstream profile [2,2,3.3,3.3,6.3,6.3,6.3] rad/s"
            )
    if not 0.001 <= args.max_command_step < 0.03:
        parser.error("--max-command-step must be at least 0.001 and below 0.03 rad")
    if not 0.5 <= args.max_joint_acceleration <= 20.0:
        parser.error(
            "--max-joint-acceleration must be between 0.5 and 20.0 rad/s^2"
        )
    if not 0.005 <= args.gripper_velocity_limit <= 0.2:
        parser.error("--gripper-velocity-limit must be between 0.005 and 0.2 rad/s")
    if not 0.05 <= args.max_tracking_error <= 0.3:
        parser.error("--max-tracking-error must be between 0.05 and 0.3 rad")

    gripper_velocity_limit = float(args.gripper_velocity_limit)
    if args.max_gripper_step is not None:
        if not 0.0001 <= args.max_gripper_step <= 0.003:
            parser.error("--max-gripper-step must be between 0.0001 and 0.003 rad")
        gripper_velocity_limit = args.max_gripper_step * args.command_hz
        print(
            "[hardware] --max-gripper-step is deprecated; using equivalent "
            f"J8 velocity {gripper_velocity_limit:.4f} rad/s",
            flush=True,
        )
    return velocity_limits, gripper_velocity_limit


class BimanualHardwareBridge:
    """Explicitly enabled, rate-limited two-arm physical driver bridge."""

    def __init__(
        self,
        config_path: pathlib.Path,
        command_hz: float,
        joint_velocity_limits: np.ndarray,
        max_tracking_error: float,
        joint_limit_margin: float = 0.05,
        max_command_step: float = 0.028,
        gripper_velocity_limit: float = 0.04,
        max_joint_acceleration: float = 6.0,
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
        self.command_hz = float(command_hz)
        self.command_period = 1.0 / self.command_hz
        self.joint_velocity_limits = np.asarray(
            joint_velocity_limits, dtype=np.float64
        )
        if (
            self.joint_velocity_limits.shape != (7,)
            or not np.all(np.isfinite(self.joint_velocity_limits))
            or np.any(self.joint_velocity_limits <= 0.0)
        ):
            raise ValueError("joint velocity limits must be seven positive values")
        self.max_command_step = float(max_command_step)
        self.gripper_velocity_limit = float(gripper_velocity_limit)
        self.max_joint_acceleration = float(max_joint_acceleration)
        self.max_tracking_error = float(max_tracking_error)
        if not 0.0 < self.max_command_step < 0.03:
            raise ValueError("max command step must be positive and below 0.03 rad")
        if self.gripper_velocity_limit <= 0.0:
            raise ValueError("gripper velocity limit must be positive")
        if self.max_joint_acceleration <= 0.0:
            raise ValueError("joint acceleration limit must be positive")
        self.last_command_time = {"left": 0.0, "right": 0.0}
        self.next_command_time = {"left": 0.0, "right": 0.0}
        self.last_command_velocity = {
            "left": np.zeros(8, dtype=np.float64),
            "right": np.zeros(8, dtype=np.float64),
        }
        self.command_intervals = {
            "left": deque(maxlen=120),
            "right": deque(maxlen=120),
        }
        self.peak_tracking_error = {"left": 0.0, "right": 0.0}
        self.last_cadence_log = time.monotonic()
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
            self.next_command_time = {"left": 0.0, "right": 0.0}
            for side in self.last_command_velocity:
                self.last_command_velocity[side].fill(0.0)
                self.command_intervals[side].clear()
                self.peak_tracking_error[side] = 0.0
            self.last_cadence_log = time.monotonic()
            limits = ",".join(f"{value:.2f}" for value in self.joint_velocity_limits)
            print(
                "[hardware] BOTH ARMS ENABLED at measured posture; "
                f"J1-J7 velocity limits=[{limits}] rad/s",
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
            details.append(
                f"{side}={actual_hz:.1f}Hz/peak_err="
                f"{self.peak_tracking_error[side]:.4f}rad"
            )
            self.peak_tracking_error[side] = 0.0
        print(
            f"[hardware] cadence target={self.command_hz:.1f}Hz "
            + " ".join(details),
            flush=True,
        )
        self.last_cadence_log = now

    def clamp_to_command_limits(
        self, side: str, desired: np.ndarray
    ) -> np.ndarray:
        """Project a command inside the soft range before the hard checker."""

        desired = np.asarray(desired, dtype=np.float64)
        limits = self.command_limits[side]
        return np.clip(desired, limits[:, 0], limits[:, 1])

    def limit_command(
        self,
        side: str,
        previous: np.ndarray,
        desired: np.ndarray,
        dt_s: float | None = None,
    ) -> np.ndarray:
        """Apply time-based per-joint velocity and acceleration limits."""

        interval = self.command_period if dt_s is None else float(dt_s)
        if not np.isfinite(interval) or interval <= 0.0:
            raise ValueError("command interval must be finite and positive")
        interval = min(interval, MAX_COMMAND_DT_S)
        previous = np.asarray(previous, dtype=np.float64)
        desired = np.asarray(desired, dtype=np.float64)
        requested_delta = desired - previous
        requested_velocity = requested_delta[:7] / interval
        velocity_limited = np.clip(
            requested_velocity,
            -self.joint_velocity_limits,
            self.joint_velocity_limits,
        )
        prior_velocity = self.last_command_velocity[side][:7]
        max_velocity_change = self.max_joint_acceleration * interval
        arm_velocity = np.clip(
            velocity_limited,
            prior_velocity - max_velocity_change,
            prior_velocity + max_velocity_change,
        )
        reversing = arm_velocity * requested_velocity < 0.0
        arm_velocity[reversing] = 0.0
        arm_delta = np.clip(
            arm_velocity * interval,
            -self.max_command_step,
            self.max_command_step,
        )
        overshooting = np.abs(arm_delta) > np.abs(requested_delta[:7])
        arm_delta[overshooting] = requested_delta[:7][overshooting]
        gripper_delta_limit = min(
            self.gripper_velocity_limit * interval, self.max_command_step
        )
        result = previous.copy()
        result[:7] += arm_delta
        result[7] += float(
            np.clip(requested_delta[7], -gripper_delta_limit, gripper_delta_limit)
        )
        return result

    def send_if_due(
        self,
        side: str,
        desired: np.ndarray,
        validator: Callable[[np.ndarray], None] | None = None,
    ) -> np.ndarray | None:
        now = time.monotonic()
        if not self.armed or not self.command_due(side, now):
            return None
        driver = self.drivers[side]
        desired = self.clamp_to_command_limits(side, desired)
        self._check_limits(side, desired)
        previous = driver.last_command.copy()
        if self.last_command_time[side] <= 0.0:
            command_dt = self.command_period
        else:
            command_dt = max(now - self.last_command_time[side], 1e-6)
        if command_dt > 2.0 * self.command_period:
            self.last_command_velocity[side].fill(0.0)
            self.last_command_time[side] = 0.0
            self.next_command_time[side] = now
            self.command_intervals[side].clear()
            command_dt = self.command_period
        command_dt = min(command_dt, MAX_COMMAND_DT_S)
        limited = self.limit_command(side, previous, desired, command_dt)
        if validator is not None:
            validator(limited)
        driver.send_position(limited)
        # SingleArmDriver.send_position() already receives a fresh motor state.
        # A second refresh here doubled CAN transactions and made the UI-owned
        # command cadence uneven.
        latest_state = driver.latest_state
        if not isinstance(latest_state, dict) or "qpos" not in latest_state:
            raise RuntimeError(f"lost {side} feedback after position command")
        measured = np.asarray(latest_state["qpos"], dtype=np.float64)
        if measured.shape != (8,) or not np.all(np.isfinite(measured)):
            raise RuntimeError(f"lost {side} feedback")
        tracking_error = np.abs(measured - limited)
        self.peak_tracking_error[side] = max(
            self.peak_tracking_error[side], float(np.max(tracking_error))
        )
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
        self.last_command_velocity[side] = (limited - previous) / command_dt
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
                measured = self.hardware.send_if_due(
                    side,
                    desired,
                    validator=lambda next_command, command_side=side: (
                        self._validate_physical_path(
                            command_side,
                            next_command,
                            physical=physical,
                            verbose=False,
                        )
                    ),
                )
                if measured is not None:
                    physical[side] = self.hardware.drivers[side].last_command.copy()
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
    add_hardware_motion_arguments(parser)
    parser.add_argument("--scale", type=float, default=0.6)
    parser.add_argument("--max-offset", type=float, default=0.15)
    args = parser.parse_args()
    if not args.confirm_hardware:
        parser.error("physical mode requires --confirm-hardware")
    velocity_limits, gripper_velocity_limit = resolve_hardware_motion_arguments(
        args, parser
    )
    bridge = BimanualHardwareBridge(
        config_path=args.config,
        command_hz=args.command_hz,
        joint_velocity_limits=velocity_limits,
        max_tracking_error=args.max_tracking_error,
        max_command_step=args.max_command_step,
        gripper_velocity_limit=gripper_velocity_limit,
        max_joint_acceleration=args.max_joint_acceleration,
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
