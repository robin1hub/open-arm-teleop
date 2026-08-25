#!/usr/bin/env python3
"""Validate chest-front position while preserving current EE orientation."""

import time
import pathlib

import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, IKParams, Kinematics
from openarm_driver import Config, SingleArmDriver


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]


TARGET_X = 0.280
TARGET_Z = 1.200


def main():
    cfg = Config(PROJECT_ROOT / "config/openarm_safe_current.yaml")
    right = SingleArmDriver("right_arm", cfg)
    left = SingleArmDriver("left_arm", cfg)
    qr = right.fetch_position(refresh=True)
    ql = left.fetch_position(refresh=True)
    right.openarm.disable_all()
    left.openarm.disable_all()

    setup = ArmSetup.from_args(
        xml=openarm_mujoco.openarm_cell_xml(), mode="right",
        frame_right="right_ee_control_point", frame_type_right="site",
        frame_left="left_ee_control_point", frame_type_left="site", keyframe="home",
    )
    kin = Kinematics(
        setup,
        IKParams(max_iters=15, dt=0.08, damping=0.15, posture_cost=0.0,
                 orientation_cost=1.0, lm_damping=0.02),
    )
    start = kin.fk("right", qr)
    target = start.copy()
    target[0] = TARGET_X
    target[2] = TARGET_Z
    kin.sync(np.concatenate([qr, ql]).astype(np.float32))

    trajectory = []
    previous = np.concatenate([qr, ql])
    max_step = 0.0
    for alpha in np.linspace(0.0, 1.0, 501)[1:]:
        pose = start + alpha * (target - start)
        kin.set_target("right", pose)
        result = kin.solve()
        if result is None:
            raise RuntimeError(f"IK failed at alpha={alpha:.3f}")
        max_step = max(max_step, float(np.max(np.abs(result - previous))))
        trajectory.append(result[:8].copy())
        previous = result
    trajectory = np.asarray(trajectory)
    actual = kin.fk("right", trajectory[-1])
    limits = np.asarray(cfg.get_joint_limits("right_arm"))
    within = np.all((trajectory >= limits[:, 0]) & (trajectory <= limits[:, 1]))
    delta = trajectory[-1] - qr
    travel = np.max(np.abs(trajectory - qr), axis=0)

    print("Current EE:", np.array2string(start, precision=5))
    print("Target  EE:", np.array2string(target, precision=5))
    print("Actual  EE:", np.array2string(actual, precision=5))
    print("Current joints deg:", np.array2string(np.degrees(qr), precision=2))
    print("Target  joints deg:", np.array2string(np.degrees(trajectory[-1]), precision=2))
    print("Delta joints deg  :", np.array2string(np.degrees(delta), precision=2))
    print("Max travel deg    :", np.array2string(np.degrees(travel), precision=2))
    print(f"Position error: {np.linalg.norm(actual[:3]-target[:3])*1000:.3f} mm")
    print(f"Max step: {max_step:.6f} rad")
    print("Within limits:", bool(within))

    if not within:
        raise RuntimeError("trajectory exceeds driver limits")
    if max_step > 0.03:
        raise RuntimeError("trajectory step exceeds 0.03 rad")
    if np.any(travel[:7] > np.radians(20.0)):
        raise RuntimeError("a joint travels more than 20 degrees")
    if delta[1] < np.radians(-10.0):
        raise RuntimeError("J2 moves backward by more than 10 degrees")
    if np.linalg.norm(actual[:3] - target[:3]) > 0.005:
        raise RuntimeError("position error exceeds 5 mm")
    print("SUCCESS: corrected chest-position trajectory is safe for real test.")


if __name__ == "__main__":
    main()
