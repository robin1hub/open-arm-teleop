#!/usr/bin/env python3
"""Stress strict IK with unreachable targets and verify hard joint limits."""

import mujoco
import numpy as np
from openarm_control import ArmSetup, IKParams

from interactive_mujoco_ee_drag import MODEL_SPECS
from safe_kinematics import SafeKinematics


def joint_ranges(setup: ArmSetup, side: str) -> np.ndarray:
    ids = [
        mujoco.mj_name2id(
            setup.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            f"openarm_{side}_joint{number}",
        )
        for number in range(1, 8)
    ]
    return setup.model.jnt_range[ids]


def main() -> int:
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
    kin = SafeKinematics(
        setup,
        IKParams(
            max_iters=15,
            dt=0.08,
            damping=0.1,
            posture_cost=0.03,
            orientation_cost=0.15,
            lm_damping=0.01,
        ),
    )
    right = np.zeros(8, dtype=np.float32)
    left = np.zeros(8, dtype=np.float32)
    kin.sync(np.r_[right, left])
    pose_right, pose_left = kin.fk_bimanual(right, left)
    unreachable = pose_right.copy()
    unreachable[:3] += np.array([2.0, -1.0, 2.0])
    ranges = joint_ranges(setup, "right")
    solved_cycles = 0
    max_step = 0.0

    for _ in range(200):
        previous = right.copy()
        kin.set_orientation_cost("right", 0.15)
        kin.set_orientation_cost("left", 1.0)
        kin.set_target("right", unreachable)
        kin.set_target("left", pose_left)
        result = kin.solve()
        if result is None:
            break
        right = result[:8]
        left = result[8:]
        solved_cycles += 1
        max_step = max(max_step, float(np.max(np.abs(right[:7] - previous[:7]))))
        if np.any(right[:7] < ranges[:, 0] - 1e-7) or np.any(
            right[:7] > ranges[:, 1] + 1e-7
        ):
            raise RuntimeError("strict IK crossed a v1 joint limit")

    print(f"bounded solve cycles before rejection/stop: {solved_cycles}")
    print(f"maximum joint step: {max_step:.6f} rad")
    print("final right joints:", np.array2string(right[:7], precision=6))
    print("lower limits:", np.array2string(ranges[:, 0], precision=6))
    print("upper limits:", np.array2string(ranges[:, 1], precision=6))
    print("SUCCESS: unreachable-target stress test stayed inside all joint limits.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
