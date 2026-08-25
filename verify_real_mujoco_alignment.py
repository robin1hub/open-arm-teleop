#!/usr/bin/env python3
"""Read-only verification that physical driver qpos maps exactly into MuJoCo."""

import time

import mujoco
import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, Kinematics
from openarm_driver import Config, SingleArmDriver


def stable_position(driver: SingleArmDriver) -> np.ndarray:
    samples = []
    for _ in range(10):
        samples.append(driver.fetch_position(refresh=True))
        time.sleep(0.02)
    samples = np.asarray(samples)
    if not np.all(np.isfinite(samples)):
        raise RuntimeError("non-finite physical feedback")
    if np.max(np.ptp(samples[-5:], axis=0)) > 0.01:
        raise RuntimeError("unstable physical feedback")
    return samples[-1]


def main() -> int:
    cfg = Config("openarm_safe_current.yaml")
    right = SingleArmDriver("right_arm", cfg)
    left = SingleArmDriver("left_arm", cfg)
    try:
        qright = stable_position(right)
        qleft = stable_position(left)
    finally:
        right.openarm.disable_all()
        left.openarm.disable_all()

    model = mujoco.MjModel.from_xml_path(openarm_mujoco.openarm_demo_xml())
    data = mujoco.MjData(model)
    resolver = openarm_mujoco.JointResolver(model)
    resolver.set_qpos(data.qpos, qright, "right")
    resolver.set_qpos(data.qpos, qleft, "left")
    mujoco.mj_forward(model, data)
    sim_right, sim_right_gripper = resolver.get_driver(data.qpos, "right")
    sim_left, sim_left_gripper = resolver.get_driver(data.qpos, "left")
    sim_right = np.append(sim_right, sim_right_gripper)
    sim_left = np.append(sim_left, sim_left_gripper)

    right_error = sim_right - qright
    left_error = sim_left - qleft

    setup = ArmSetup.from_args(
        xml=openarm_mujoco.openarm_cell_xml(), mode="bimanual",
        frame_right="right_ee_control_point", frame_type_right="site",
        frame_left="left_ee_control_point", frame_type_left="site", keyframe="home",
    )
    kin = Kinematics(setup)
    pose_right, pose_left = kin.fk_bimanual(qright, qleft)

    print("Physical right qpos:", np.array2string(qright, precision=6))
    print("MuJoCo   right qpos:", np.array2string(sim_right, precision=6))
    print("Physical left  qpos:", np.array2string(qleft, precision=6))
    print("MuJoCo   left  qpos:", np.array2string(sim_left, precision=6))
    print("Right qpos error:", np.array2string(right_error, precision=9))
    print("Left  qpos error:", np.array2string(left_error, precision=9))
    print("Right EE pose [xyz,qwxyz]:", np.array2string(pose_right, precision=6))
    print("Left  EE pose [xyz,qwxyz]:", np.array2string(pose_left, precision=6))

    if np.max(np.abs(right_error)) > 1e-6:
        raise RuntimeError("right physical->MuJoCo mapping error exceeds 1e-6 rad")
    if np.max(np.abs(left_error)) > 1e-6:
        raise RuntimeError("left physical->MuJoCo mapping error exceeds 1e-6 rad")
    print("SUCCESS: physical qpos and MuJoCo qpos are exactly aligned.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
