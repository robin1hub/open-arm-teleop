#!/usr/bin/env python3
"""Headless synthetic VIVE trajectory test using official driver checks."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass
import math
import pathlib
import sys
import time
from types import SimpleNamespace

import numpy as np
import mujoco
import mujoco.viewer

ROOT = pathlib.Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from openarm_control import ArmSetup, IKParams
from openarm_driver import Config
from openarm_driver.safety import (
    JointDeltaPosChecker,
    JointPosChecker,
    JointVelocityChecker,
)

from openarm_teleop.interactive_mujoco_ee_drag import (
    MODEL_SPECS,
    axis_angle_quat,
    quat_error_deg,
    quat_multiply,
)
from openarm_teleop.safe_kinematics import SafeKinematics


@dataclass(frozen=True)
class Scenario:
    name: str
    duration_s: float
    xyz_amplitude_m: tuple[float, float, float]
    wrist_amplitude_deg: float


@dataclass
class Metrics:
    frames: int
    elapsed_s: float
    solve_failures: int
    position_clips: int
    delta_stops: int
    velocity_limited_commands: int
    max_candidate_jump_rad: float
    peak_velocity_rad_s: np.ndarray
    position_errors_m: list[float]
    orientation_errors_deg: list[float]

    @property
    def effective_hz(self) -> float:
        return self.frames / max(self.elapsed_s, 1e-9)


def build_kinematics() -> tuple[
    ArmSetup, SafeKinematics, np.ndarray, tuple[np.ndarray, np.ndarray]
]:
    spec = MODEL_SPECS["v1"]
    setup = ArmSetup.from_args(
        xml=str(spec["xml"]),
        mode="bimanual",
        frame_right=spec["frames"]["right"],
        frame_type_right=spec["frame_type"],
        frame_left=spec["frames"]["left"],
        frame_type_left=spec["frame_type"],
        keyframe=spec["keyframe"],
    )
    kinematics = SafeKinematics(
        setup,
        IKParams(
            max_iters=15,
            dt=0.08,
            damping=0.1,
            posture_cost=0.03,
            orientation_cost=1.0,
            lm_damping=0.01,
        ),
    )
    # Match the physical VIVE path: the official driver, not IK, owns motion
    # velocity and single-command delta checks.
    kinematics.set_max_joint_change_per_solve(np.inf)

    right7, right_gripper = setup.joint_resolver.get_driver(
        setup.data.qpos, "right"
    )
    left7, left_gripper = setup.joint_resolver.get_driver(
        setup.data.qpos, "left"
    )
    right = np.r_[right7, right_gripper].astype(np.float64)
    left = np.r_[left7, left_gripper].astype(np.float64)
    right[3] = math.radians(45.0)
    left[3] = math.radians(45.0)
    command = np.r_[right, left]
    kinematics.sync(command.astype(np.float32))
    anchors = kinematics.fk_bimanual(
        right.astype(np.float32), left.astype(np.float32)
    )
    return setup, kinematics, command, anchors


def trajectory_target(
    anchor: np.ndarray,
    phase: float,
    side_sign: float,
    scenario: Scenario,
) -> np.ndarray:
    x_amp, y_amp, z_amp = scenario.xyz_amplitude_m
    target = anchor.copy()
    target[:3] += np.array(
        [
            x_amp * math.sin(phase),
            side_sign * y_amp * math.sin(2.0 * phase),
            z_amp * (1.0 - math.cos(phase)),
        ],
        dtype=np.float64,
    )
    wrist_delta = axis_angle_quat(
        np.array([0.0, 1.0, 0.0]),
        math.radians(scenario.wrist_amplitude_deg) * math.sin(phase),
    )
    target[3:] = quat_multiply(wrist_delta, anchor[3:])
    return target


def run_scenario(
    scenario: Scenario, command_hz: float, show_viewer: bool = False
) -> Metrics:
    setup, kinematics, command, anchors = build_kinematics()
    config = Config("openarm_cell")
    velocity_limits = np.asarray(
        config.get_joint_velocity_limits(), dtype=np.float64
    )
    delta_limits = np.asarray(
        config.get_joint_delta_position_limits(), dtype=np.float64
    )
    drivers = {
        "right": SimpleNamespace(last_command=command[:8].copy()),
        "left": SimpleNamespace(last_command=command[8:].copy()),
    }
    checkers = {
        side: (
            JointPosChecker(config.get_joint_limits(f"{side}_arm")),
            JointDeltaPosChecker(delta_limits),
            JointVelocityChecker(velocity_limits),
        )
        for side in drivers
    }

    frames = int(round(scenario.duration_s * command_hz)) + 1
    position_errors: list[float] = []
    orientation_errors: list[float] = []
    peak_velocity = np.zeros(8, dtype=np.float64)
    solve_failures = 0
    position_clips = 0
    delta_stops = 0
    velocity_limited_commands = 0
    max_candidate_jump = 0.0
    compute_elapsed = 0.0
    processed_frames = 0
    viewer_context = (
        mujoco.viewer.launch_passive(setup.model, setup.data)
        if show_viewer
        else nullcontext(None)
    )
    with viewer_context as viewer:
        replay_started = time.monotonic()
        if viewer is not None:
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -18.0
            viewer.cam.distance = 1.7

        for frame_index, phase in enumerate(
            np.linspace(0.0, 2.0 * np.pi, frames)
        ):
            if viewer is not None and not viewer.is_running():
                break
            compute_started = time.perf_counter()
            targets = (
                trajectory_target(anchors[0], phase, -1.0, scenario),
                trajectory_target(anchors[1], phase, 1.0, scenario),
            )
            kinematics.sync(command.astype(np.float32))
            kinematics.set_target("right", targets[0])
            kinematics.set_target("left", targets[1])
            solution = kinematics.solve()
            if solution is None:
                solve_failures += 1
                continue

            candidate = solution.astype(np.float64)
            max_candidate_jump = max(
                max_candidate_jump,
                float(np.max(np.abs(candidate - command))),
            )
            next_command = command.copy()
            for side, offset in (("right", 0), ("left", 8)):
                desired = candidate[offset : offset + 8]
                driver = drivers[side]
                position_checker, delta_checker, velocity_checker = checkers[side]

                position_result = position_checker.check(desired)
                position_clips += int(not position_result.is_safe)
                if position_result.fixed_joint_positions is not None:
                    desired = np.asarray(
                        position_result.fixed_joint_positions, dtype=np.float64
                    )

                delta_result = delta_checker.check(desired, driver=driver)
                if not delta_result.is_safe:
                    delta_stops += 1
                    if delta_result.force_stop:
                        raise RuntimeError(
                            f"{scenario.name} frame {frame_index}: "
                            f"{delta_result.message}"
                        )

                velocity_result = velocity_checker.check(
                    desired, driver=driver, dt_s=1.0 / command_hz
                )
                velocity_limited_commands += int(not velocity_result.is_safe)
                accepted = (
                    np.asarray(
                        velocity_result.fixed_joint_positions, dtype=np.float64
                    )
                    if velocity_result.fixed_joint_positions is not None
                    else desired.copy()
                )
                peak_velocity = np.maximum(
                    peak_velocity,
                    np.abs(accepted - driver.last_command) * command_hz,
                )
                driver.last_command = accepted.copy()
                next_command[offset : offset + 8] = accepted

            command = next_command
            actual_right, actual_left = kinematics.fk_bimanual(
                command[:8].astype(np.float32), command[8:].astype(np.float32)
            )
            for actual, target in zip((actual_right, actual_left), targets):
                position_errors.append(
                    float(np.linalg.norm(actual[:3] - target[:3]))
                )
                orientation_errors.append(quat_error_deg(actual[3:], target[3:]))
            compute_elapsed += time.perf_counter() - compute_started
            processed_frames += 1

            if viewer is not None:
                setup.joint_resolver.set_qpos(
                    setup.data.qpos, command[:8], "right"
                )
                setup.joint_resolver.set_qpos(
                    setup.data.qpos, command[8:], "left"
                )
                mujoco.mj_forward(setup.model, setup.data)
                viewer.sync()
                deadline = replay_started + (frame_index + 1) / command_hz
                time.sleep(max(0.0, deadline - time.monotonic()))

    return Metrics(
        frames=processed_frames,
        elapsed_s=compute_elapsed,
        solve_failures=solve_failures,
        position_clips=position_clips,
        delta_stops=delta_stops,
        velocity_limited_commands=velocity_limited_commands,
        max_candidate_jump_rad=max_candidate_jump,
        peak_velocity_rad_s=peak_velocity,
        position_errors_m=position_errors,
        orientation_errors_deg=orientation_errors,
    )


def report_and_check(
    scenario: Scenario, metrics: Metrics, command_hz: float
) -> None:
    position = np.asarray(metrics.position_errors_m)
    orientation = np.asarray(metrics.orientation_errors_deg)
    print(f"[{scenario.name}] frames={metrics.frames} at {command_hz:.1f} Hz")
    print(
        f"  compute={metrics.elapsed_s:.3f}s "
        f"({metrics.effective_hz:.1f} simulated frames/s)"
    )
    print(
        f"  failures: IK={metrics.solve_failures} "
        f"position_clips={metrics.position_clips} "
        f"delta_stops={metrics.delta_stops}"
    )
    print(
        f"  official velocity-limited arm-frames="
        f"{metrics.velocity_limited_commands}"
    )
    print(
        f"  position RMS/max={1000.0 * np.sqrt(np.mean(position**2)):.3f}/"
        f"{1000.0 * np.max(position):.3f} mm"
    )
    print(
        f"  orientation RMS/max={np.sqrt(np.mean(orientation**2)):.3f}/"
        f"{np.max(orientation):.3f} deg"
    )
    print(f"  max IK command jump={metrics.max_candidate_jump_rad:.4f} rad")
    print(
        "  peak joint velocity="
        + np.array2string(metrics.peak_velocity_rad_s, precision=3)
        + " rad/s"
    )

    official_limits = np.array([2.0, 2.0, 3.3, 3.3, 6.3, 6.3, 6.3, 20.0])
    failures = []
    if metrics.solve_failures:
        failures.append("IK failure")
    if metrics.position_clips:
        failures.append("official position clip")
    if metrics.delta_stops:
        failures.append("official delta stop")
    if metrics.effective_hz < command_hz:
        failures.append("slower than real time")
    if np.any(metrics.peak_velocity_rad_s > official_limits + 1e-6):
        failures.append("official velocity exceeded")
    if float(np.max(position)) > 0.015:
        failures.append("position error exceeded 15 mm")
    if float(np.max(orientation)) > 5.0:
        failures.append("orientation error exceeded 5 deg")
    if failures:
        raise RuntimeError(f"{scenario.name} failed: {', '.join(failures)}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command-hz", type=float, default=60.0)
    parser.add_argument(
        "--scenario", choices=("all", "nominal", "fast"), default="all"
    )
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    if not 20.0 <= args.command_hz <= 100.0:
        parser.error("--command-hz must be between 20 and 100")

    scenarios = (
        Scenario("nominal", 6.0, (0.04, 0.025, 0.035), 15.0),
        Scenario("fast", 1.5, (0.06, 0.04, 0.05), 25.0),
    )
    selected = (
        scenarios
        if args.scenario == "all"
        else tuple(item for item in scenarios if item.name == args.scenario)
    )
    for scenario in selected:
        metrics = run_scenario(scenario, args.command_hz, args.viewer)
        report_and_check(scenario, metrics, args.command_hz)
    print("SUCCESS: synthetic VIVE trajectories passed official-driver simulation.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
