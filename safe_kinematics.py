"""Limit-preserving differential IK for the interactive OpenArm controller.

Unlike the upstream solver version currently installed in this workspace,
this solver never retries without joint limits.  It also anchors redundant
solutions to the configuration at the start of every solve cycle.
"""

from __future__ import annotations

import mink
import mink.exceptions
import mujoco
import numpy as np
from openarm_control import ArmSetup, IKParams, Kinematics
from openarm_control.poses import pose_to_se3


class SafeKinematics(Kinematics):
    """Kinematics API-compatible wrapper with a strict IK implementation."""

    def __init__(self, setup: ArmSetup, params: IKParams) -> None:
        super().__init__(setup, ik_params=None)
        self._ik = _StrictIKSolver(setup, params)

    def set_orientation_cost(self, side: str, cost: float) -> None:
        self._require_ik().set_orientation_cost(side, cost)


class _StrictIKSolver:
    def __init__(self, setup: ArmSetup, params: IKParams) -> None:
        self._setup = setup
        self._sides = setup.sides
        self._solver_name = params.solver
        self._joint_resolver = setup.joint_resolver
        self._dt = params.dt
        self._max_iters = params.max_iters
        self._posture_cost = params.posture_cost
        self._config = mink.Configuration(setup.model)
        self._config.update(q=setup.data.qpos.copy())

        task_kwargs = {
            "position_cost": params.position_cost,
            "orientation_cost": params.orientation_cost,
            "lm_damping": params.lm_damping,
        }
        self._tasks = {
            side: mink.FrameTask(
                frame_name=_frame_name(setup, side),
                frame_type=setup.frame_types[side],
                **task_kwargs,
            )
            for side in setup.sides
        }

        active_qpos = set(setup.joint_resolver._right.arm_qpos.tolist()) | set(
            setup.joint_resolver._left.arm_qpos.tolist()
        )
        freeze_dofs = [
            int(setup.model.jnt_dofadr[joint_id])
            for joint_id in range(setup.model.njnt)
            if setup.model.jnt_qposadr[joint_id] not in active_qpos
        ]
        self._freeze_task = (
            mink.DofFreezingTask(model=setup.model, dof_indices=freeze_dofs)
            if freeze_dofs
            else None
        )
        self._limits = [mink.ConfigurationLimit(setup.model)]
        if params.velocity_limits is not None:
            self._limits.append(mink.VelocityLimit(setup.model, params.velocity_limits))

        self._posture_task = mink.PostureTask(
            setup.model, cost=max(params.posture_cost, 1e-6)
        )
        self._posture_task.set_target(self._config.q)
        self._solver_params = {"damping": params.damping}
        if params.diag_reg > 0.0:
            self._solver_params["diag_reg"] = params.diag_reg

        self._pending = set(setup.sides)
        self._gripper = np.zeros(2, dtype=np.float32)
        self.last_failure = ""
        self._max_joint_change_per_solve = 0.06

    def set_target(self, side: str, pose: np.ndarray) -> None:
        self._tasks[side].set_target(pose_to_se3(pose))
        self._pending.discard(side)

    def set_orientation_cost(self, side: str, cost: float) -> None:
        if side not in self._tasks:
            raise ValueError(f"unknown arm side: {side}")
        self._tasks[side].set_orientation_cost(float(cost))

    def sync(self, values16: np.ndarray) -> None:
        qpos = self._config.data.qpos.copy()
        self._joint_resolver.set_qpos(qpos, values16[:8], "right")
        self._joint_resolver.set_qpos(qpos, values16[8:16], "left")
        self._config.update(q=qpos)
        self._posture_task.set_target(qpos)
        self._gripper[:] = (values16[7], values16[15])

    def ready(self) -> bool:
        return not self._pending

    def solve(self) -> np.ndarray | None:
        # Anchor this cycle's redundant solution to its starting posture.
        self._posture_task.set_target(self._config.q)
        tasks = list(self._tasks.values())
        if self._posture_cost > 0.0:
            tasks.append(self._posture_task)
        constraints = [self._freeze_task] if self._freeze_task else []
        start_q = self._config.q.copy()

        for _ in range(self._max_iters):
            try:
                velocity = mink.solve_ik(
                    self._config,
                    tasks,
                    self._dt,
                    self._solver_name,
                    limits=self._limits,
                    constraints=constraints,
                    # ConfigurationLimit remains in the QP.  Mink's pre-solve
                    # safety break is disabled because v1 zero starts exactly
                    # at J4's lower bound and roundoff reports 0 as a violation.
                    # The integrated result is hard-checked below.
                    safety_break=False,
                    **self._solver_params,
                )
            except (
                mink.exceptions.NoSolutionFound,
                mink.exceptions.NotWithinConfigurationLimits,
            ) as error:
                self._config.update(q=start_q)
                self._pending = set(self._sides)
                self.last_failure = f"{type(error).__name__}: {error}"
                return None
            self._config.integrate_inplace(velocity, self._dt)

        if not self._enforce_joint_limits():
            self._config.update(q=start_q)
            self._pending = set(self._sides)
            self.last_failure = "integrated result is outside MuJoCo joint limits"
            return None
        if self._max_arm_change(start_q) > self._max_joint_change_per_solve:
            change = self._max_arm_change(start_q)
            self._config.update(q=start_q)
            self._pending = set(self._sides)
            self.last_failure = (
                f"joint branch jump {change:.4f} rad exceeds "
                f"{self._max_joint_change_per_solve:.4f} rad"
            )
            return None

        self._pending = set(self._sides)
        self.last_failure = ""
        right, _ = self._joint_resolver.get_driver(self._config.q, "right")
        left, _ = self._joint_resolver.get_driver(self._config.q, "left")
        return np.concatenate(
            [
                np.r_[right, self._gripper[0]],
                np.r_[left, self._gripper[1]],
            ]
        ).astype(np.float32)

    def _enforce_joint_limits(self) -> bool:
        """Clip numerical boundary noise; reject any material limit violation."""
        model = self._setup.model
        qpos = self._config.q.copy()
        clipped = False
        for side in ("right", "left"):
            for number in range(1, 8):
                joint_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    f"openarm_{side}_joint{number}",
                )
                if joint_id < 0 or not model.jnt_limited[joint_id]:
                    continue
                qpos_address = model.jnt_qposadr[joint_id]
                low, high = model.jnt_range[joint_id]
                value = qpos[qpos_address]
                if value < low:
                    if low - value > 1e-5:
                        return False
                    qpos[qpos_address] = low
                    clipped = True
                elif value > high:
                    if value - high > 1e-5:
                        return False
                    qpos[qpos_address] = high
                    clipped = True
        if clipped:
            self._config.update(q=qpos)
        return True

    def _max_arm_change(self, start_q: np.ndarray) -> float:
        indices = np.r_[
            self._joint_resolver._right.arm_qpos,
            self._joint_resolver._left.arm_qpos,
        ]
        return float(np.max(np.abs(self._config.q[indices] - start_q[indices])))


def _frame_name(setup: ArmSetup, side: str) -> str:
    object_type = {
        "body": mujoco.mjtObj.mjOBJ_BODY,
        "site": mujoco.mjtObj.mjOBJ_SITE,
        "geom": mujoco.mjtObj.mjOBJ_GEOM,
    }[setup.frame_types[side]]
    return mujoco.mj_id2name(
        setup.model, object_type, setup.frame_ids[side]
    )
