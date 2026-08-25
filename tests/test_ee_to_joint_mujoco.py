#!/usr/bin/env python3
"""Headless MuJoCo EE-pose -> IK -> joint-angle -> FK validation."""

import math

import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, IKParams, Kinematics


def quat_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Return the shortest angular difference between qwxyz quaternions."""
    dot = float(np.clip(abs(np.dot(a, b)), 0.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


def main() -> int:
    setup = ArmSetup.from_args(
        xml=openarm_mujoco.openarm_cell_xml(),
        mode="bimanual",
        frame_right="right_ee_control_point",
        frame_type_right="site",
        frame_left="left_ee_control_point",
        frame_type_left="site",
        keyframe="home",
    )
    kin = Kinematics(
        setup,
        IKParams(
            max_iters=10,
            dt=0.1,
            damping=0.1,
            posture_cost=0.01,
            lm_damping=0.01,
        ),
    )

    right7, right_gripper = setup.joint_resolver.get_driver(setup.data.qpos, "right")
    left7, left_gripper = setup.joint_resolver.get_driver(setup.data.qpos, "left")
    right0 = np.append(right7, right_gripper).astype(np.float32)
    left0 = np.append(left7, left_gripper).astype(np.float32)
    pose0_right, pose0_left = kin.fk_bimanual(right0, left0)

    target_right = pose0_right.copy()
    target_left = pose0_left.copy()
    target_right[2] += 0.05
    target_left[2] += 0.05

    kin.sync(np.concatenate([right0, left0]).astype(np.float32))
    result = None
    max_joint_step = 0.0
    previous = np.concatenate([right0, left0])

    # Feed a smooth 100-sample Cartesian path, just like a VR pose stream.
    for alpha in np.linspace(0.0, 1.0, 101)[1:]:
        pose_right = pose0_right + alpha * (target_right - pose0_right)
        pose_left = pose0_left + alpha * (target_left - pose0_left)
        kin.set_target("right", pose_right)
        kin.set_target("left", pose_left)
        result = kin.solve()
        if result is None or result.shape != (16,) or not np.all(np.isfinite(result)):
            raise RuntimeError(f"IK failed at alpha={alpha:.3f}")
        max_joint_step = max(max_joint_step, float(np.max(np.abs(result - previous))))
        previous = result.copy()

    assert result is not None
    solved_right = result[:8]
    solved_left = result[8:]
    actual_right, actual_left = kin.fk_bimanual(solved_right, solved_left)

    right_position_error = float(np.linalg.norm(actual_right[:3] - target_right[:3]))
    left_position_error = float(np.linalg.norm(actual_left[:3] - target_left[:3]))
    right_orientation_error = quat_error_deg(actual_right[3:], target_right[3:])
    left_orientation_error = quat_error_deg(actual_left[3:], target_left[3:])

    print("Initial right EE:", np.array2string(pose0_right, precision=5))
    print("Target  right EE:", np.array2string(target_right, precision=5))
    print("Actual  right EE:", np.array2string(actual_right, precision=5))
    print("Initial left  EE:", np.array2string(pose0_left, precision=5))
    print("Target  left  EE:", np.array2string(target_left, precision=5))
    print("Actual  left  EE:", np.array2string(actual_left, precision=5))
    print("Solved right joints:", np.array2string(solved_right, precision=5))
    print("Solved left  joints:", np.array2string(solved_left, precision=5))
    print(f"Right position/orientation error: {right_position_error:.6f} m / "
          f"{right_orientation_error:.4f} deg")
    print(f"Left  position/orientation error: {left_position_error:.6f} m / "
          f"{left_orientation_error:.4f} deg")
    print(f"Maximum joint change per Cartesian sample: {max_joint_step:.6f} rad")

    if max(right_position_error, left_position_error) > 0.005:
        raise RuntimeError("position error exceeds 5 mm")
    if max(right_orientation_error, left_orientation_error) > 1.0:
        raise RuntimeError("orientation error exceeds 1 degree")
    if max_joint_step > 0.03:
        raise RuntimeError("joint step exceeds physical-driver 0.03 rad limit")
    print("SUCCESS: EE pose -> IK -> joints -> MuJoCo FK passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
