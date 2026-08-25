#!/usr/bin/env python3
"""Real bimanual 5 cm +X and +Z EE motion through OpenArm IK and drivers."""

import time

import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, IKParams, Kinematics
from openarm_driver import Config, SingleArmDriver


MAX_TORQUES = np.array([12.0, 12.0, 4.0, 12.0, 2.0, 2.0, 2.0, 2.0])
MAX_TOTAL_TRAVEL = np.array([0.30, 0.20, 0.30, 0.35, 0.30, 0.30, 0.30, 0.10])
STEPS = 250  # 5 seconds at 50 Hz


def make_kinematics() -> Kinematics:
    setup = ArmSetup.from_args(
        xml=openarm_mujoco.openarm_cell_xml(),
        mode="bimanual",
        frame_right="right_ee_control_point",
        frame_type_right="site",
        frame_left="left_ee_control_point",
        frame_type_left="site",
        keyframe="home",
    )
    return Kinematics(
        setup,
        IKParams(
            orientation_cost=0.1,
            max_iters=10,
            dt=0.1,
            damping=0.1,
            posture_cost=0.0,
            lm_damping=0.01,
        ),
    )


def plan(qright: np.ndarray, qleft: np.ndarray):
    kin = make_kinematics()
    start_right, start_left = kin.fk_bimanual(qright, qleft)
    target_right = start_right.copy()
    target_left = start_left.copy()
    target_right[0] += 0.05
    target_left[0] += 0.05
    target_right[2] += 0.05
    target_left[2] += 0.05
    kin.sync(np.concatenate([qright, qleft]).astype(np.float32))

    trajectory = []
    previous = np.concatenate([qright, qleft])
    max_step = 0.0
    for alpha in np.linspace(0.0, 1.0, STEPS + 1)[1:]:
        pose_right = start_right + alpha * (target_right - start_right)
        pose_left = start_left + alpha * (target_left - start_left)
        kin.set_target("right", pose_right)
        kin.set_target("left", pose_left)
        result = kin.solve()
        if result is None or result.shape != (16,) or not np.all(np.isfinite(result)):
            raise RuntimeError(f"IK failed at alpha={alpha:.3f}")
        step = float(np.max(np.abs(result - previous)))
        max_step = max(max_step, step)
        if step > 0.03:
            raise RuntimeError(
                f"IK joint step {step:.5f} rad exceeds 0.03 at alpha={alpha:.3f}"
            )
        trajectory.append(result.copy())
        previous = result
    return np.asarray(trajectory), start_right, start_left, target_right, target_left, max_step


def main() -> int:
    cfg = Config("openarm_safe_current.yaml")
    drivers = {
        "right": SingleArmDriver("right_arm", cfg),
        "left": SingleArmDriver("left_arm", cfg),
    }
    started = []
    try:
        measured = {}
        for side, driver in drivers.items():
            samples = []
            for _ in range(10):
                samples.append(driver.fetch_position(refresh=True))
                time.sleep(0.02)
            samples = np.asarray(samples)
            if not np.all(np.isfinite(samples)):
                raise RuntimeError(f"{side}: invalid initial feedback")
            if np.max(np.ptp(samples[-5:], axis=0)) > 0.01:
                raise RuntimeError(f"{side}: unstable initial feedback")
            measured[side] = samples[-1].copy()

        # Full offline preflight before enabling.
        trajectory, pose_r, pose_l, target_r, target_l, max_step = plan(
            measured["right"], measured["left"]
        )
        limits = {
            side: np.asarray(cfg.get_joint_limits(side + "_arm"))
            for side in ("right", "left")
        }
        for side, sl in (("right", slice(0, 8)), ("left", slice(8, 16))):
            q = trajectory[:, sl]
            below = np.min(q, axis=0) < limits[side][:, 0]
            above = np.max(q, axis=0) > limits[side][:, 1]
            if np.any(below | above):
                joints = np.where(below | above)[0]
                details = [
                    (
                        f"J{j+1}: planned [{np.min(q[:,j]):.5f},"
                        f"{np.max(q[:,j]):.5f}], allowed "
                        f"[{limits[side][j,0]:.5f},{limits[side][j,1]:.5f}]"
                    )
                    for j in joints
                ]
                raise RuntimeError(
                    f"{side}: planned IK trajectory exceeds driver limits: "
                    + "; ".join(details)
                )
            total_travel = np.max(np.abs(q - measured[side]), axis=0)
            if np.any(total_travel > MAX_TOTAL_TRAVEL):
                j = int(np.argmax(total_travel / MAX_TOTAL_TRAVEL))
                raise RuntimeError(
                    f"{side}: planned J{j+1} travel {total_travel[j]:.4f} rad "
                    f"exceeds {MAX_TOTAL_TRAVEL[j]:.4f} rad"
                )
        print("Right EE start/target:", pose_r[:3], target_r[:3], flush=True)
        print("Left  EE start/target:", pose_l[:3], target_l[:3], flush=True)
        print(f"Planned max joint step: {max_step:.6f} rad", flush=True)
        print(
            "Planned max joint travel right:",
            np.max(np.abs(trajectory[:, :8] - measured["right"]), axis=0),
            flush=True,
        )
        print(
            "Planned final joint delta right deg:",
            np.degrees(trajectory[-1, :8] - measured["right"]),
            flush=True,
        )
        print(
            "Planned max joint travel left :",
            np.max(np.abs(trajectory[:, 8:] - measured["left"]), axis=0),
            flush=True,
        )
        print(
            "Planned final joint delta left  deg:",
            np.degrees(trajectory[-1, 8:] - measured["left"]),
            flush=True,
        )

        # Enable and immediately anchor each driver to a fresh measured pose.
        origins = {}
        previous = {}
        peak = {side: np.zeros(8) for side in drivers}
        for side in ("right", "left"):
            drivers[side].last_command = measured[side].copy()
            drivers[side].start()
            started.append(side)
            origins[side] = drivers[side].fetch_position(refresh=True).copy()
            drivers[side].last_command = origins[side].copy()
            previous[side] = origins[side].copy()
            if np.max(np.abs(origins[side] - measured[side])) > 0.02:
                raise RuntimeError(f"{side}: pose changed by >0.02 rad while enabling")

        def command(qright: np.ndarray, qleft: np.ndarray, phase: str) -> None:
            for side, qcmd in (("right", qright), ("left", qleft)):
                driver = drivers[side]
                driver.send_position(qcmd)
                state = driver.fetch_state(refresh=True)
                q, torque = state["qpos"], state["qtorque"]
                if not np.all(np.isfinite(q)) or not np.all(np.isfinite(torque)):
                    raise RuntimeError(f"{side} {phase}: non-finite state")
                if np.any(np.abs(q - previous[side]) > 0.05):
                    raise RuntimeError(f"{side} {phase}: feedback jump")
                if np.any(np.abs(qcmd - q) > 0.18):
                    raise RuntimeError(f"{side} {phase}: following error {qcmd-q}")
                if np.any(np.abs(torque) > MAX_TORQUES):
                    j = int(np.argmax(np.abs(torque) / MAX_TORQUES))
                    raise RuntimeError(
                        f"{side} {phase}: J{j+1} torque {torque[j]:.3f} Nm "
                        f"exceeds {MAX_TORQUES[j]:.1f} Nm"
                    )
                previous[side] = q.copy()
                peak[side] = np.maximum(peak[side], np.abs(torque))
            time.sleep(0.02)

        for _ in range(50):
            command(origins["right"], origins["left"], "takeover")

        for row in trajectory:
            command(row[:8], row[8:], "up")
        print("Reached +X 5 cm / +Z 5 cm Cartesian target.", flush=True)

        for _ in range(100):
            command(trajectory[-1, :8], trajectory[-1, 8:], "hold")

        for row in trajectory[::-1]:
            command(row[:8], row[8:], "return")
        for _ in range(50):
            command(origins["right"], origins["left"], "final hold")

        for side in ("right", "left"):
            error = previous[side] - origins[side]
            print(f"{side} return error:", np.array2string(error, precision=5), flush=True)
            print(
                f"{side} peak torque:", np.array2string(peak[side], precision=3), flush=True
            )
            if np.any(np.abs(error) > 0.025):
                raise RuntimeError(f"{side}: return error exceeds 0.025 rad")
        print("SUCCESS: real bimanual EE forward/up 5 cm motion completed.", flush=True)
        return 0
    finally:
        for side in reversed(started):
            try:
                drivers[side].stop()
            except Exception:
                try:
                    drivers[side].openarm.disable_all()
                except Exception:
                    pass
        for side in drivers:
            if side not in started:
                try:
                    drivers[side].openarm.disable_all()
                except Exception:
                    pass
        print("Safety cleanup: both arms disabled.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
