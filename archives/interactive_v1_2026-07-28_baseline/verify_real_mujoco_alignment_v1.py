#!/usr/bin/env python3
"""Read-only verification of this OpenArm 1.0 physical-to-MuJoCo mapping."""

import pathlib
import time

import mujoco
import numpy as np
from openarm_driver import Config, SingleArmDriver

from interactive_mujoco_ee_drag import physical_to_model_position
from openarm_mujoco.v2 import JointResolver


ROOT = pathlib.Path(__file__).resolve().parent
MODEL = ROOT / "models" / "openarm_v1" / "scene.xml"
CONFIG = ROOT / "openarm_safe_raw_zero.yaml"


def stable_position(driver: SingleArmDriver) -> np.ndarray:
    samples = []
    for _ in range(10):
        samples.append(driver.fetch_position(refresh=True))
        time.sleep(0.02)
    samples = np.asarray(samples, dtype=np.float64)
    if not np.all(np.isfinite(samples)):
        raise RuntimeError("non-finite physical feedback")
    if float(np.max(np.ptp(samples[-5:], axis=0))) > 0.01:
        raise RuntimeError("unstable physical feedback")
    return samples[-1]


def main() -> int:
    config = Config(CONFIG)
    drivers = {
        "right": SingleArmDriver("right_arm", config),
        "left": SingleArmDriver("left_arm", config),
    }
    try:
        physical = {side: stable_position(driver) for side, driver in drivers.items()}
    finally:
        for driver in drivers.values():
            driver.openarm.disable_all()

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    resolver = JointResolver(model)
    model_position = {
        side: physical_to_model_position(side, position, "v1")
        for side, position in physical.items()
    }
    for side, position in model_position.items():
        resolver.set_qpos(data.qpos, position, side)
    mujoco.mj_forward(model, data)

    for side in ("right", "left"):
        joints, gripper = resolver.get_driver(data.qpos, side)
        roundtrip = np.r_[joints, gripper]
        arm_error = roundtrip[:7] - physical[side][:7]
        joint_ids = [
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{side}_joint{i}"
            )
            for i in range(1, 8)
        ]
        ranges = model.jnt_range[joint_ids]
        violations = (joints < ranges[:, 0]) | (joints > ranges[:, 1])
        print(f"{side} physical [J1..J7, gripper rad]:")
        print(np.array2string(physical[side], precision=6))
        print(f"{side} MuJoCo [J1..J7, finger slide m]:")
        print(np.array2string(roundtrip, precision=6))
        print(f"{side} arm roundtrip error:")
        print(np.array2string(arm_error, precision=9))
        if np.max(np.abs(arm_error)) > 1e-8:
            raise RuntimeError(f"{side} arm mapping is not exact")
        if np.any(violations):
            raise RuntimeError(
                f"{side} joints outside v1 MJCF limits: "
                f"{np.flatnonzero(violations).tolist()}"
            )

    print("SUCCESS: OpenArm 1.0 J1-J7 physical/MuJoCo mapping is exact.")
    print("INFO: v1 gripper rotor radians were converted to 0..0.044 m finger travel.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
