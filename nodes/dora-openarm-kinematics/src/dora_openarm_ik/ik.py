# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Dora node: mink-based differential IK solver for OpenArm.

Accepts end-effector pose targets and solves joint angles via mink's QP-based
differential IK. Both arms share one mink.Configuration and one QP solve per
step.

Pose convention (inputs and outputs):  float32[7] = [px, py, pz, qw, qx, qy, qz]
Inputs:
  target_right – float32[7]  right EE target pose
  target_left  – float32[7]  left  EE target pose
  position     – float32[16] current joint state right[8]+left[8] (optional sync)
  state_right  – struct{qpos,qvel,qtorque} or float32[8] right real-arm state
  state_left   – struct{qpos,qvel,qtorque} or float32[8] left real-arm state

Outputs:
  position_right – float32[8] solved right arm joint angles
  position_left  – float32[8] solved left arm joint angles
  status         – ["ready"] on startup
"""

from __future__ import annotations

import argparse
import time

import dora
import numpy as np
import pyarrow as pa

from openarm_control import (
    Kinematics,
    register_common_args,
    register_ik_args,
    ik_params_from_args,
    setup_from_args,
)


def _map_trigger_to_gripper(trigger: float, side: str) -> float:
    """trigger 0.0~1.0 → gripper angle"""
    if side == "right":
        return (-1.57 / 2.0) * (1.0 - trigger)  # 0→-1.57, 1→0
    else:
        return (1.57 / 2.0) * (1.0 - trigger)   # 0→ 1.57, 1→0


def _position_values(value) -> np.ndarray:
    """Extract float32 joint positions from a plain array or follower state."""
    if isinstance(value, pa.StructArray):
        if "qpos" not in value.type.names:
            return np.array([], dtype=np.float32)
        value = value.field("qpos")
    return np.asarray(value, dtype=np.float32)


def _run(args: argparse.Namespace) -> None:
    kin = Kinematics(setup_from_args(args), ik_params_from_args(args))
    real_state: dict[str, np.ndarray] = {}

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    for event in node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        values = _position_values(event["value"])

        if eid == "position":
            if values.shape == (16,):
                kin.sync(values)
            continue
        if eid in ("state_right", "state_left"):
            side = eid.removeprefix("state_")
            if values.shape != (8,):
                print(f"Warning: expected {eid} qpos[8], got {values.shape}. Skipping.")
                continue
            real_state[side] = values.copy()
            if "right" in real_state and "left" in real_state:
                kin.sync(np.concatenate([real_state["right"], real_state["left"]]))
            continue

        if eid == "target_right" and "right" in kin.setup.sides:
            if values.shape != (7,):
                print(f"Warning: expected target_right[7], got {values.shape}. Skipping.")
                continue
            kin.set_target("right", values)

        elif eid == "target_left" and "left" in kin.setup.sides:
            if values.shape != (7,):
                print(f"Warning: expected target_left[7], got {values.shape}. Skipping.")
                continue
            kin.set_target("left", values)

        elif eid == "trigger_right":
            kin.set_gripper("right", _map_trigger_to_gripper(float(values[0]), "right"))
            continue

        elif eid == "trigger_left":
            kin.set_gripper("left", _map_trigger_to_gripper(float(values[0]), "left"))
            continue

        else:
            continue

        if not kin.ready():
            continue

        result = kin.solve()
        if result is None:
            continue

        ts = {"timestamp": time.time_ns()}
        node.send_output("position_right", pa.array(result[:8],  type=pa.float32()), ts)
        node.send_output("position_left",  pa.array(result[8:16], type=pa.float32()), ts)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mink IK dora node – OpenArm end-effector pose → joint angles"
    )
    register_common_args(parser)
    register_ik_args(parser)
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()
