#!/usr/bin/env python3
"""Guarded real right-arm move to a validated chest-front grasp pose."""

import time

import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, IKParams, Kinematics
from openarm_driver import Config, SingleArmDriver


TARGET_X = 0.280
TARGET_Z = 1.200
MAX_TORQUES = np.array([12.0, 12.0, 4.0, 12.0, 2.0, 2.0, 2.0, 2.0])
STEPS = 600


def make_kinematics() -> Kinematics:
    setup = ArmSetup.from_args(
        xml=openarm_mujoco.openarm_cell_xml(), mode="right",
        frame_right="right_ee_control_point", frame_type_right="site",
        frame_left="left_ee_control_point", frame_type_left="site", keyframe="home",
    )
    return Kinematics(
        setup,
        IKParams(max_iters=15, dt=0.08, damping=0.15, posture_cost=0.0,
                 orientation_cost=1.0, lm_damping=0.02),
    )


def normalized_pose_lerp(start: np.ndarray, target: np.ndarray, alpha: float):
    pose = start + alpha * (target - start)
    pose[3:] /= np.linalg.norm(pose[3:])
    return pose


def plan(qright: np.ndarray, qleft: np.ndarray):
    kin = make_kinematics()
    start = kin.fk("right", qright)
    target = start.copy()
    target[0] = TARGET_X
    target[2] = TARGET_Z
    kin.sync(np.concatenate([qright, qleft]).astype(np.float32))
    trajectory = []
    previous = np.concatenate([qright, qleft])
    max_step = 0.0
    for alpha in np.linspace(0.0, 1.0, STEPS + 1)[1:]:
        kin.set_target("right", normalized_pose_lerp(start, target, alpha))
        result = kin.solve()
        if result is None or not np.all(np.isfinite(result)):
            raise RuntimeError(f"IK failed at alpha={alpha:.3f}")
        step = float(np.max(np.abs(result - previous)))
        if step > 0.03:
            raise RuntimeError(f"IK step {step:.5f} exceeds 0.03 rad")
        max_step = max(max_step, step)
        trajectory.append(result[:8].copy())
        previous = result
    actual = kin.fk("right", trajectory[-1])
    return np.asarray(trajectory), start, target, actual, max_step


def main() -> int:
    cfg = Config("openarm_safe_current.yaml")
    right = SingleArmDriver("right_arm", cfg)
    left = SingleArmDriver("left_arm", cfg)
    started = False
    try:
        samples = []
        for _ in range(10):
            samples.append(right.fetch_position(refresh=True))
            time.sleep(0.02)
        qright = np.asarray(samples)[-1]
        qleft = left.fetch_position(refresh=True)
        if np.max(np.ptp(np.asarray(samples)[-5:], axis=0)) > 0.01:
            raise RuntimeError("unstable initial right-arm feedback")

        trajectory, start_pose, target_pose, actual_pose, max_step = plan(qright, qleft)
        limits = np.asarray(cfg.get_joint_limits("right_arm"))
        if np.any(trajectory < limits[:, 0]) or np.any(trajectory > limits[:, 1]):
            raise RuntimeError("planned trajectory exceeds right-arm limits")
        position_error = float(np.linalg.norm(actual_pose[:3] - target_pose[:3]))
        if position_error > 0.005:
            raise RuntimeError(f"planned EE position error {position_error:.5f} m")
        print("EE start :", np.array2string(start_pose, precision=5), flush=True)
        print("EE target:", np.array2string(target_pose, precision=5), flush=True)
        print("EE solved:", np.array2string(actual_pose, precision=5), flush=True)
        print("Target joints deg:", np.degrees(trajectory[-1]), flush=True)
        print("Joint delta deg :", np.degrees(trajectory[-1] - qright), flush=True)
        print(f"Max planned step: {max_step:.6f} rad", flush=True)
        travel = np.max(np.abs(trajectory - qright), axis=0)
        travel_caps = np.radians([20, 10, 20, 10, 30, 20, 20, 5])
        if np.any(travel > travel_caps):
            j = int(np.argmax(travel / travel_caps))
            raise RuntimeError(
                f"J{j+1} planned travel {np.degrees(travel[j]):.2f} deg exceeds "
                f"{np.degrees(travel_caps[j]):.2f} deg"
            )

        right.last_command = qright.copy()
        right.start()
        started = True
        origin = right.fetch_position(refresh=True).copy()
        right.last_command = origin.copy()
        if np.max(np.abs(origin - qright)) > 0.02:
            raise RuntimeError("right-arm pose changed by >0.02 rad while enabling")

        previous = origin.copy()
        peak = np.zeros(8)

        def command(qcmd: np.ndarray, phase: str) -> None:
            nonlocal previous, peak
            right.send_position(qcmd)
            state = right.fetch_state(refresh=True)
            q, torque = state["qpos"], state["qtorque"]
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(torque)):
                raise RuntimeError(f"{phase}: non-finite state")
            if np.any(np.abs(q - previous) > 0.05):
                raise RuntimeError(f"{phase}: feedback jump {q-previous}")
            if np.any(np.abs(qcmd - q) > 0.15):
                raise RuntimeError(f"{phase}: following error {qcmd-q}")
            if np.any(np.abs(torque) > MAX_TORQUES):
                j = int(np.argmax(np.abs(torque) / MAX_TORQUES))
                raise RuntimeError(
                    f"{phase}: J{j+1} torque {torque[j]:.3f} Nm exceeds "
                    f"{MAX_TORQUES[j]:.1f} Nm"
                )
            previous = q.copy()
            peak = np.maximum(peak, np.abs(torque))
            time.sleep(0.025)

        for _ in range(40):
            command(origin, "takeover")
        for qcmd in trajectory:
            command(qcmd, "to chest")
        print("Reached chest-front grasp pose.", flush=True)
        print("Reached joints deg:", np.degrees(previous), flush=True)
        for _ in range(80):
            command(trajectory[-1], "chest hold")
        for qcmd in trajectory[::-1]:
            command(qcmd, "return")
        for _ in range(40):
            command(origin, "final hold")

        error = previous - origin
        print("Return error:", np.array2string(error, precision=5), flush=True)
        print("Peak torque:", np.array2string(peak, precision=3), flush=True)
        if np.any(np.abs(error) > 0.025):
            raise RuntimeError("return error exceeds 0.025 rad")
        print("SUCCESS: chest-front grasp pose completed and returned.", flush=True)
        return 0
    finally:
        if started:
            right.stop()
        else:
            try:
                right.openarm.disable_all()
            except Exception:
                pass
        try:
            left.openarm.disable_all()
        except Exception:
            pass
        print("Safety cleanup: both arms disabled.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
