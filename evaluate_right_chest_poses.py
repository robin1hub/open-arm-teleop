#!/usr/bin/env python3
"""Evaluate larger right-arm chest-front EE poses from current real state."""

import time

import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, IKParams, Kinematics
from openarm_driver import Config, SingleArmDriver


POSES = {
    "home_front": np.array([0.401, -0.1535, 1.12, 0.7071068, 0, -0.7071068, 0]),
    "chest_mid": np.array([0.32, -0.14, 1.18, 0.7071068, 0, -0.7071068, 0]),
    "chest_high": np.array([0.30, -0.14, 1.23, 0.7071068, 0, -0.7071068, 0]),
}


def make_kin():
    setup = ArmSetup.from_args(
        xml=openarm_mujoco.openarm_cell_xml(), mode="right",
        frame_right="right_ee_control_point", frame_type_right="site",
        frame_left="left_ee_control_point", frame_type_left="site", keyframe="home",
    )
    return Kinematics(
        setup,
        IKParams(max_iters=15, dt=0.08, damping=0.15, posture_cost=0.0,
                 orientation_cost=0.5, lm_damping=0.02),
    )


def main():
    cfg = Config("openarm_safe_current.yaml")
    right = SingleArmDriver("right_arm", cfg)
    left = SingleArmDriver("left_arm", cfg)
    qr = right.fetch_position(refresh=True)
    ql = left.fetch_position(refresh=True)
    right.openarm.disable_all(); left.openarm.disable_all()
    limits = np.asarray(cfg.get_joint_limits("right_arm"))

    for name, target in POSES.items():
        kin = make_kin()
        start = kin.fk("right", qr)
        kin.sync(np.concatenate([qr, ql]).astype(np.float32))
        previous = np.concatenate([qr, ql])
        max_step = 0.0
        result = None
        for alpha in np.linspace(0, 1, 401)[1:]:
            pose = start + alpha * (target - start)
            quat = pose[3:]
            pose[3:] = quat / np.linalg.norm(quat)
            kin.set_target("right", pose)
            result = kin.solve()
            if result is None:
                break
            max_step = max(max_step, float(np.max(np.abs(result - previous))))
            previous = result
        if result is None:
            print(name, "IK_FAILED")
            continue
        actual = kin.fk("right", result[:8])
        pos_error = np.linalg.norm(actual[:3] - target[:3])
        within = np.all((result[:8] >= limits[:, 0]) & (result[:8] <= limits[:, 1]))
        print(f"[{name}]")
        print(" start :", np.array2string(start, precision=5))
        print(" target:", np.array2string(target, precision=5))
        print(" actual:", np.array2string(actual, precision=5))
        print(" joints deg:", np.array2string(np.degrees(result[:8]), precision=2))
        print(" delta deg :", np.array2string(np.degrees(result[:8]-qr), precision=2))
        print(f" pos_error={pos_error*1000:.2f}mm max_step={max_step:.5f}rad within={within}")


if __name__ == "__main__":
    main()
