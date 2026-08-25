#!/usr/bin/env python3
"""Drag an OpenArm end-effector target in MuJoCo (simulation only).

Controls
--------
Left drag         Move the target in the camera plane
Shift + left      Move the target horizontally / along world Z
Right drag        Rotate the target about the camera up/right axes
Alt + left drag   Rotate the camera
Middle drag       Pan the camera
Mouse wheel       Zoom
Tab               Switch active arm
R                 Reset target to the current end-effector pose
Space             Pause/resume IK following
Esc               Quit

This program intentionally imports no CAN or real-arm driver.
"""

from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import pathlib
import time

import glfw
import mujoco
import mujoco.viewer
import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, IKParams
from .safe_kinematics import SafeKinematics


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]


MODEL_SPECS = {
    "v1": {
        "xml": PROJECT_ROOT
        / "models"
        / "openarm_v1"
        / "scene.xml",
        "frames": {
            "right": "openarm_right_hand_tcp",
            "left": "openarm_left_hand_tcp",
        },
        "frame_type": "body",
        "keyframe": None,
    },
    "v2": {
        "xml": pathlib.Path(openarm_mujoco.openarm_cell_xml()),
        "frames": {
            "right": "right_ee_control_point",
            "left": "left_ee_control_point",
        },
        "frame_type": "site",
        "keyframe": "home",
    },
}


def native_control_viewer_worker(
    xml_path: str, initial_qpos: np.ndarray, connection: object
) -> None:
    """Run MuJoCo's stock Control panel in its own GLFW process."""
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    data.qpos[:] = initial_qpos
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        model.actuator_ctrllimited[actuator_id] = 1
        model.actuator_ctrlrange[actuator_id] = model.jnt_range[joint_id]
        data.ctrl[actuator_id] = data.qpos[int(model.jnt_qposadr[joint_id])]
    snapshot = data.ctrl.copy()
    mujoco.mj_forward(model, data)
    try:
        with mujoco.viewer.launch_passive(
            model, data, show_left_ui=False, show_right_ui=True
        ) as viewer:
            while viewer.is_running():
                changed = np.flatnonzero(np.abs(data.ctrl - snapshot) > 1e-7)
                user_changed = bool(changed.size)
                if changed.size:
                    for actuator_id in changed:
                        joint_id = int(model.actuator_trnid[actuator_id, 0])
                        if (
                            joint_id >= 0
                            and model.jnt_type[joint_id]
                            == mujoco.mjtJoint.mjJNT_HINGE
                        ):
                            data.qpos[int(model.jnt_qposadr[joint_id])] = data.ctrl[
                                actuator_id
                            ]
                    connection.send(("ctrl", data.ctrl.copy()))
                    snapshot = data.ctrl.copy()
                newest_qpos = None
                while connection.poll():
                    message, payload = connection.recv()
                    if message == "quit":
                        return
                    if message == "qpos":
                        newest_qpos = payload
                if newest_qpos is not None and not user_changed:
                    data.qpos[:] = newest_qpos
                    for actuator_id in range(model.nu):
                        joint_id = int(model.actuator_trnid[actuator_id, 0])
                        if (
                            joint_id >= 0
                            and model.jnt_type[joint_id]
                            == mujoco.mjtJoint.mjJNT_HINGE
                        ):
                            data.ctrl[actuator_id] = data.qpos[
                                int(model.jnt_qposadr[joint_id])
                            ]
                    snapshot = data.ctrl.copy()
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(0.01)
    except (BrokenPipeError, EOFError):
        return

TRANSLATION_ORIENTATION_COST = 0.15
ROTATION_ORIENTATION_COST = 1.0


class HardwareBridge:
    """Explicitly armed, rate-limited bridge to physical OpenArm drivers."""

    def __init__(self, config_path: pathlib.Path, command_hz: float, max_step: float):
        from openarm_driver import Config, SingleArmDriver

        config = Config(config_path)
        self.drivers = {
            "right": SingleArmDriver("right_arm", config),
            "left": SingleArmDriver("left_arm", config),
        }
        self.joint_limits = {
            side: config.get_joint_limits(f"{side}_arm") for side in self.drivers
        }
        self.command_period = 1.0 / command_hz
        self.max_step = max_step
        self.last_command_time = 0.0
        self.last_poll_time = 0.0
        self.armed_side: str | None = None
        self.move_goal: np.ndarray | None = None
        self.follow_enabled = False
        self.initial_positions = {
            side: self._stable_position(driver) for side, driver in self.drivers.items()
        }
        for side, position in self.initial_positions.items():
            limits = config.get_joint_limits(f"{side}_arm")
            if np.any(position < limits[:, 0]) or np.any(position > limits[:, 1]):
                raise RuntimeError(f"{side} initial posture is outside configured limits")
        # Construction and initial state reads must never leave motors enabled.
        for driver in self.drivers.values():
            driver.openarm.disable_all()

    @staticmethod
    def _stable_position(driver: object) -> np.ndarray:
        samples = []
        for _ in range(8):
            samples.append(driver.fetch_position(refresh=True))
            time.sleep(0.02)
        samples = np.asarray(samples, dtype=np.float64)
        if not np.all(np.isfinite(samples)):
            raise RuntimeError("non-finite physical joint feedback")
        if float(np.max(np.ptp(samples[-5:], axis=0))) > 0.02:
            raise RuntimeError("physical joint feedback is not stable")
        return samples[-1].copy()

    def arm(self, side: str) -> None:
        if self.armed_side == side:
            return
        self.disarm()
        driver = self.drivers[side]
        measured = np.asarray(driver.fetch_position(refresh=True), dtype=np.float64)
        if not np.all(np.isfinite(measured)):
            raise RuntimeError(f"cannot enable {side}: invalid physical feedback")
        limits = self.joint_limits[side]
        if np.any(measured < limits[:, 0]) or np.any(measured > limits[:, 1]):
            raise RuntimeError(f"cannot enable {side}: physical posture outside limits")
        # The arm may have been moved while disabled.  Rate limiting must start
        # from the latest measured posture, never a stale previous command.
        driver.last_command = measured.copy()
        driver.start()
        # start() refreshes last_command from measured qpos and start.moves is
        # empty in the safe config, so enabling cannot initiate a trajectory.
        self.armed_side = side
        self.move_goal = None
        self.follow_enabled = False
        self.last_command_time = 0.0
        print(f"[hardware] ARMED {side}; press D to move to simulation pose", flush=True)

    def disarm(self) -> None:
        if self.armed_side is None:
            return
        side = self.armed_side
        self.armed_side = None
        self.move_goal = None
        self.follow_enabled = False
        try:
            self.drivers[side].stop()
        finally:
            print(f"[hardware] DISABLED {side}", flush=True)

    def send_if_due(self, side: str, desired: np.ndarray) -> np.ndarray | None:
        if self.armed_side != side:
            return None
        now = time.monotonic()
        if now - self.last_command_time < self.command_period:
            return None
        driver = self.drivers[side]
        limited = limit_joint_command(driver.last_command, desired, self.max_step)
        driver.send_position(limited)
        measured = np.asarray(driver.fetch_position(refresh=True), dtype=np.float64)
        if not np.all(np.isfinite(measured)):
            self.disarm()
            raise RuntimeError("lost finite physical feedback; arm disabled")
        self.last_command_time = now
        return measured

    def command_due(self) -> bool:
        return time.monotonic() - self.last_command_time >= self.command_period

    def set_follow(self, enabled: bool) -> None:
        if enabled and self.armed_side is None:
            raise RuntimeError("enable the selected arm with E first")
        self.move_goal = None
        self.follow_enabled = enabled
        print(
            f"[hardware] continuous follow {'ON' if enabled else 'OFF'}",
            flush=True,
        )

    def set_move_goal(self, side: str, goal: np.ndarray) -> None:
        if self.armed_side != side:
            raise RuntimeError(f"{side} arm is not enabled")
        goal = np.asarray(goal, dtype=np.float64)
        if goal.shape != (8,) or not np.all(np.isfinite(goal)):
            raise ValueError("move goal must contain 8 finite joint positions")
        limits = self.joint_limits[side]
        if np.any(goal < limits[:, 0]) or np.any(goal > limits[:, 1]):
            violated = np.flatnonzero(
                (goal < limits[:, 0]) | (goal > limits[:, 1])
            ).tolist()
            raise RuntimeError(f"simulation goal violates joint limits: {violated}")
        self.follow_enabled = False
        self.move_goal = goal.copy()
        error = float(np.max(np.abs(self.move_goal - self.drivers[side].last_command)))
        print(f"[hardware] MOVE {side} to simulation; initial max error={error:.4f} rad", flush=True)

    def cancel_move(self, reason: str) -> None:
        if self.move_goal is not None:
            print(f"[hardware] MOVE cancelled: {reason}", flush=True)
        self.move_goal = None

    def poll_if_due(self) -> dict[str, np.ndarray] | None:
        """Read both arms at command_hz without enabling or sending positions."""
        now = time.monotonic()
        if now - self.last_poll_time < self.command_period:
            return None
        positions = {
            side: np.asarray(driver.fetch_position(refresh=True), dtype=np.float64)
            for side, driver in self.drivers.items()
        }
        if not all(np.all(np.isfinite(position)) for position in positions.values()):
            raise RuntimeError("lost finite physical feedback")
        self.last_poll_time = now
        return positions

    def read_positions_now(self) -> dict[str, np.ndarray]:
        positions = {
            side: np.asarray(driver.fetch_position(refresh=True), dtype=np.float64)
            for side, driver in self.drivers.items()
        }
        if not all(np.all(np.isfinite(position)) for position in positions.values()):
            raise RuntimeError("lost finite physical feedback")
        return positions

    def close(self) -> None:
        self.disarm()
        for driver in self.drivers.values():
            driver.openarm.disable_all()


def limit_joint_command(
    previous: np.ndarray,
    desired: np.ndarray,
    max_step: float | np.ndarray,
) -> np.ndarray:
    """Pure rate limiter used before the driver's own safety checker."""
    previous = np.asarray(previous, dtype=np.float64)
    desired = np.asarray(desired, dtype=np.float64)
    if previous.shape != (8,) or desired.shape != (8,):
        raise ValueError("physical joint commands must contain 8 values")
    if not np.all(np.isfinite(desired)):
        raise ValueError("physical joint command is not finite")
    return previous + np.clip(desired - previous, -max_step, max_step)


def physical_to_model_position(
    side: str, position: np.ndarray, model_version: str
) -> np.ndarray:
    """Convert physical 7-joint + gripper feedback into model coordinates."""
    converted = np.asarray(position, dtype=np.float64).copy()
    if model_version == "v1":
        # V1 hardware reports gripper rotor angle (60 degrees full stroke);
        # MJCF finger joints are linear slides with a 44 mm full stroke.
        direction = -1.0 if side == "right" else 1.0
        converted[7] = np.clip(
            direction * converted[7] * 0.044 / (math.pi / 3.0), 0.0, 0.044
        )
    return converted


def quat_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    dot = float(np.clip(abs(np.dot(a, b)), 0.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    out = np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )
    return out / np.linalg.norm(out)


def axis_angle_quat(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    half = 0.5 * angle
    return np.r_[math.cos(half), axis * math.sin(half)]


def camera_basis(camera: mujoco.MjvCamera) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return camera right, screen-up and forward vectors in world coordinates."""
    az = math.radians(camera.azimuth)
    el = math.radians(camera.elevation)
    forward = np.array(
        [-math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el)]
    )
    right = np.array([math.cos(az), math.sin(az), 0.0])
    up = np.cross(right, forward)
    return right, up / np.linalg.norm(up), forward


def rendered_camera_basis(
    scene: mujoco.MjvScene, fallback: mujoco.MjvCamera
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read the basis actually used by MuJoCo after mjv_updateScene."""
    forward = np.asarray(scene.camera[0].forward, dtype=np.float64)
    up = np.asarray(scene.camera[0].up, dtype=np.float64)
    if np.linalg.norm(forward) < 0.5 or np.linalg.norm(up) < 0.5:
        return camera_basis(fallback)
    forward /= np.linalg.norm(forward)
    up -= forward * np.dot(up, forward)
    up /= np.linalg.norm(up)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    return right, up, forward


def approach_quat(current: np.ndarray, desired: np.ndarray, max_angle: float) -> np.ndarray:
    """Move a qwxyz quaternion toward desired by no more than max_angle."""
    desired = desired.copy()
    dot = float(np.dot(current, desired))
    if dot < 0.0:
        desired = -desired
        dot = -dot
    angle = 2.0 * math.acos(float(np.clip(dot, 0.0, 1.0)))
    if angle <= max_angle:
        return desired
    alpha = max_angle / max(angle, 1e-9)
    out = (1.0 - alpha) * current + alpha * desired
    return out / np.linalg.norm(out)


def add_geom(
    scene: mujoco.MjvScene,
    geom_type: mujoco.mjtGeom,
    size: np.ndarray,
    pos: np.ndarray,
    rgba: np.ndarray,
) -> mujoco.MjvGeom | None:
    if scene.ngeom >= scene.maxgeom:
        return None
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        geom_type,
        np.asarray(size, dtype=np.float64),
        np.asarray(pos, dtype=np.float64),
        np.eye(3, dtype=np.float64).ravel(),
        np.asarray(rgba, dtype=np.float32),
    )
    scene.ngeom += 1
    return geom


def quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    mat = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, dtype=np.float64))
    return mat.reshape(3, 3)


def add_target_frame(
    scene: mujoco.MjvScene,
    pose: np.ndarray,
    reachable: bool,
    scale: float = 0.08,
    active: bool = True,
) -> None:
    if active:
        center_color = (
            np.array([0.1, 0.9, 0.25, 0.9])
            if reachable
            else np.array([1.0, 0.1, 0.1, 0.95])
        )
    else:
        center_color = np.array([0.65, 0.65, 0.65, 0.4])
    add_geom(
        scene,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([0.014, 0.014, 0.014]),
        pose[:3],
        center_color,
    )
    rotation = quat_to_matrix(pose[3:])
    colors = (
        (
            np.array([1.0, 0.1, 0.1, 1.0]),
            np.array([0.1, 1.0, 0.1, 1.0]),
            np.array([0.1, 0.35, 1.0, 1.0]),
        )
        if active
        else (
            np.array([0.65, 0.35, 0.35, 0.35]),
            np.array([0.35, 0.65, 0.35, 0.35]),
            np.array([0.35, 0.45, 0.65, 0.35]),
        )
    )
    for axis, color in zip(rotation.T, colors):
        geom = add_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.ones(3),
            pose[:3],
            color,
        )
        if geom is not None:
            mujoco.mjv_connector(
                geom,
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                0.004,
                pose[:3],
                pose[:3] + scale * axis,
            )


class DragApp:
    def __init__(
        self, side: str, model_version: str, hardware: HardwareBridge | None = None
    ) -> None:
        self.hardware = hardware
        self.model_version = model_version
        spec = MODEL_SPECS[model_version]
        self.setup = ArmSetup.from_args(
            xml=str(spec["xml"]),
            mode="bimanual",
            frame_right=spec["frames"]["right"],
            frame_type_right=spec["frame_type"],
            frame_left=spec["frames"]["left"],
            frame_type_left=spec["frame_type"],
            keyframe=spec["keyframe"],
        )
        self.kin = SafeKinematics(
            self.setup,
            IKParams(
                max_iters=15,
                dt=0.08,
                damping=0.1,
                posture_cost=0.03,
                orientation_cost=TRANSLATION_ORIENTATION_COST,
                lm_damping=0.01,
            ),
        )
        self.model = self.setup.model
        self.data = self.setup.data
        self.resolver = self.setup.joint_resolver
        self.side = side
        self.follow = True
        self.reachable = True
        self.last_xy = (0.0, 0.0)
        self.position_error = 0.0
        self.orientation_error = 0.0
        if self.hardware is not None:
            self.resolver.set_qpos(
                self.data.qpos,
                physical_to_model_position(
                    "right", self.hardware.initial_positions["right"], model_version
                ),
                "right",
            )
            self.resolver.set_qpos(
                self.data.qpos,
                physical_to_model_position(
                    "left", self.hardware.initial_positions["left"], model_version
                ),
                "left",
            )
            mujoco.mj_forward(self.model, self.data)
        self._sync_from_model()
        self.target = self._fk(self.side).copy()
        self.ik_target = self.target.copy()
        self.last_log_time = 0.0
        self.last_reject_reason = ""
        self.orientation_strict_until = 0.0
        self.target_position_step = 0.0025
        self.target_orientation_step = math.radians(1.0)
        self.strict_orientation_cost = ROTATION_ORIENTATION_COST

        if not glfw.init():
            raise RuntimeError("GLFW initialization failed")
        self.window = glfw.create_window(1280, 800, "OpenArm MuJoCo EE drag", None, None)
        if self.window is None:
            glfw.terminate()
            raise RuntimeError("GLFW window creation failed")
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        self.camera = mujoco.MjvCamera()
        self.option = mujoco.MjvOption()
        # OpenArm visual meshes live in geom group 2.  Show only that group:
        # groups 0/1/4 contain the surrounding cell/table that can occlude it,
        # while group 3 contains simplified collision shapes.
        self.option.geomgroup[:] = 0
        self.option.geomgroup[2] = 1
        self.option.sitegroup[:] = 0
        self.scene = mujoco.MjvScene(self.model, maxgeom=2000)
        self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        mujoco.mj_forward(self.model, self.data)
        visual_ids = np.flatnonzero(self.model.geom_group == 2)
        visual_xyz = self.data.geom_xpos[visual_ids]
        visual_min = visual_xyz.min(axis=0)
        visual_max = visual_xyz.max(axis=0)
        self.camera.lookat[:] = 0.5 * (visual_min + visual_max)
        self.camera.distance = max(1.25, 2.2 * float(np.max(visual_max - visual_min)))
        self.camera.azimuth = 135.0
        self.camera.elevation = -18.0

        glfw.set_key_callback(self.window, self._key)
        glfw.set_cursor_pos_callback(self.window, self._cursor)
        glfw.set_mouse_button_callback(self.window, self._mouse_button)
        glfw.set_scroll_callback(self.window, self._scroll)

        # Keep MuJoCo's stock viewer in a separate process. Two GLFW event
        # loops in one process make one or both windows unresponsive.
        process_context = mp.get_context("spawn")
        self.native_connection, child_connection = process_context.Pipe()
        self.native_process = process_context.Process(
            target=native_control_viewer_worker,
            args=(str(spec["xml"]), self.data.qpos.copy(), child_connection),
            daemon=True,
        )
        self.native_process.start()
        child_connection.close()
        self.last_native_activity = 0.0
        self.last_native_sync = 0.0

    def _drivers(self) -> tuple[np.ndarray, np.ndarray]:
        r7, rg = self.resolver.get_driver(self.data.qpos, "right")
        l7, lg = self.resolver.get_driver(self.data.qpos, "left")
        return np.r_[r7, rg].astype(np.float32), np.r_[l7, lg].astype(np.float32)

    def _sync_from_model(self) -> None:
        right, left = self._drivers()
        self.kin.sync(np.r_[right, left].astype(np.float32))

    def _fk(self, side: str) -> np.ndarray:
        right, left = self._drivers()
        return np.asarray(self.kin.fk(side, right if side == "right" else left))

    def _apply_native_controls(self) -> None:
        newest_ctrl = None
        try:
            while self.native_connection.poll():
                message, payload = self.native_connection.recv()
                if message == "ctrl":
                    newest_ctrl = np.asarray(payload, dtype=np.float64)
        except (BrokenPipeError, EOFError):
            return
        if newest_ctrl is None:
            return
        self.last_native_activity = time.monotonic()
        changed = np.arange(min(self.model.nu, newest_ctrl.size))
        changed_sides: set[str] = set()
        for actuator_id in changed:
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            if joint_id < 0 or self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            qpos_address = int(self.model.jnt_qposadr[joint_id])
            self.data.qpos[qpos_address] = newest_ctrl[actuator_id]
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
            ) or ""
            if "openarm_right_" in name:
                changed_sides.add("right")
            elif "openarm_left_" in name:
                changed_sides.add("left")
        if changed_sides:
            mujoco.mj_forward(self.model, self.data)
            self._sync_from_model()
            if self.side in changed_sides:
                self.target = self._fk(self.side).copy()
                self.ik_target = self.target.copy()
                self.reachable = True
                self.last_reject_reason = ""

    def _sync_native_controls(self) -> None:
        now = time.monotonic()
        if now - self.last_native_sync < 0.05 or not self.native_process.is_alive():
            return
        try:
            self.native_connection.send(("qpos", self.data.qpos.copy()))
            self.last_native_sync = now
        except (BrokenPipeError, EOFError):
            pass

    def _mouse_button(self, _window: glfw._GLFWwindow, _button: int, _action: int, _mods: int) -> None:
        self.last_xy = glfw.get_cursor_pos(self.window)

    def _cursor(self, _window: glfw._GLFWwindow, xpos: float, ypos: float) -> None:
        if not any(
            glfw.get_mouse_button(self.window, button) == glfw.PRESS
            for button in (glfw.MOUSE_BUTTON_LEFT, glfw.MOUSE_BUTTON_RIGHT, glfw.MOUSE_BUTTON_MIDDLE)
        ):
            self.last_xy = (xpos, ypos)
            return
        width, height = glfw.get_window_size(self.window)
        dx = (xpos - self.last_xy[0]) / max(height, 1)
        dy = (ypos - self.last_xy[1]) / max(height, 1)
        self.last_xy = (xpos, ypos)
        alt = bool(
            glfw.get_key(self.window, glfw.KEY_LEFT_ALT) == glfw.PRESS
            or glfw.get_key(self.window, glfw.KEY_RIGHT_ALT) == glfw.PRESS
        )
        shift = bool(
            glfw.get_key(self.window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
            or glfw.get_key(self.window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        )
        left = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        right_button = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        middle = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
        cam_right, cam_up, _ = rendered_camera_basis(self.scene, self.camera)

        if left and not alt:
            gain = max(0.25, self.camera.distance) * 1.25
            if shift:
                delta = gain * (dx * cam_right - dy * np.array([0.0, 0.0, 1.0]))
            else:
                delta = gain * (dx * cam_right - dy * cam_up)
            self.target[:3] += delta
        elif right_button and not alt:
            q_up = axis_angle_quat(cam_up, -3.0 * dx)
            q_right = axis_angle_quat(cam_right, -3.0 * dy)
            self.target[3:] = quat_multiply(q_right, quat_multiply(q_up, self.target[3:]))
            self.orientation_strict_until = time.monotonic() + 0.3
        else:
            if left:
                action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
            elif middle:
                action = mujoco.mjtMouse.mjMOUSE_MOVE_V
            else:
                return
            mujoco.mjv_moveCamera(self.model, action, dx, dy, self.scene, self.camera)

    def _scroll(self, _window: glfw._GLFWwindow, _xoffset: float, yoffset: float) -> None:
        mujoco.mjv_moveCamera(
            self.model,
            mujoco.mjtMouse.mjMOUSE_ZOOM,
            0.0,
            -0.05 * yoffset,
            self.scene,
            self.camera,
        )

    def _key(self, _window: glfw._GLFWwindow, key: int, _scancode: int, action: int, _mods: int) -> None:
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(self.window, True)
        elif key == glfw.KEY_SPACE:
            self.follow = not self.follow
        elif key == glfw.KEY_E and self.hardware is not None:
            if self.hardware.armed_side == self.side:
                self.hardware.disarm()
            else:
                self.hardware.arm(self.side)
        elif key == glfw.KEY_F and self.hardware is not None:
            if self.hardware.armed_side != self.side:
                print("[hardware] F ignored: select a side and press E first", flush=True)
            else:
                self.hardware.set_follow(not self.hardware.follow_enabled)
        elif key == glfw.KEY_D and self.hardware is not None:
            if self.hardware.armed_side != self.side:
                print("[hardware] D ignored: select a side and press E first", flush=True)
            elif not self.reachable:
                print("[hardware] D ignored: simulation target is not reachable", flush=True)
            elif float(np.linalg.norm(self.target[:3] - self.ik_target[:3])) > 0.005:
                print("[hardware] D ignored: wait for simulation IK to settle", flush=True)
            else:
                right, left = self._drivers()
                goal = right if self.side == "right" else left
                if self.model_version == "v1":
                    # No v1 gripper UI yet: preserve its physical motor angle.
                    goal = goal.copy()
                    goal[7] = self.hardware.drivers[self.side].last_command[7]
                try:
                    self._validate_physical_path(self.side, goal)
                    self.hardware.set_move_goal(self.side, goal)
                except (RuntimeError, ValueError) as error:
                    print(f"[hardware] D ignored: {error}", flush=True)
        elif key == glfw.KEY_P and self.hardware is not None:
            try:
                self._sync_simulation_to_physical()
            except RuntimeError as error:
                print(f"[hardware] P failed: {error}", flush=True)
        elif key == glfw.KEY_TAB:
            self._select_side("left" if self.side == "right" else "right")
        elif key == glfw.KEY_1:
            self._select_side("left")
        elif key == glfw.KEY_2:
            self._select_side("right")
        elif key == glfw.KEY_R:
            self.target = self._fk(self.side).copy()
            self.ik_target = self.target.copy()
            self.reachable = True

    def _select_side(self, side: str) -> None:
        if side == self.side:
            return
        if self.hardware is not None:
            self.hardware.disarm()
        self.side = side
        # Begin at the selected arm's current pose.  An old target is
        # deliberately discarded so switching can never create a jump.
        self.target = self._fk(side).copy()
        self.ik_target = self.target.copy()
        self.reachable = True
        self.last_reject_reason = ""
        print(f"[select] active arm={side}", flush=True)

    def _sync_simulation_to_physical(self) -> None:
        assert self.hardware is not None
        self.hardware.disarm()
        positions = self.hardware.read_positions_now()
        for side, physical in positions.items():
            self.hardware.drivers[side].last_command = physical.copy()
            model_position = physical_to_model_position(
                side, physical, self.model_version
            )
            self.resolver.set_qpos(self.data.qpos, model_position, side)
        mujoco.mj_forward(self.model, self.data)
        self._sync_from_model()
        self.target = self._fk(self.side).copy()
        self.ik_target = self.target.copy()
        self.reachable = True
        self.last_reject_reason = ""
        print("[hardware] simulation resynchronized to physical arms; motors disabled", flush=True)

    def _validate_physical_path(
        self,
        side: str,
        goal: np.ndarray,
        physical: dict[str, np.ndarray] | None = None,
        verbose: bool = True,
    ) -> None:
        """Check limits and active-arm collisions along a joint-linear path."""
        assert self.hardware is not None
        if physical is None:
            physical = self.hardware.read_positions_now()
        start = physical[side]
        limits = self.hardware.joint_limits[side]
        if np.any(goal < limits[:, 0]) or np.any(goal > limits[:, 1]):
            violated = np.flatnonzero(
                (goal < limits[:, 0]) | (goal > limits[:, 1])
            ).tolist()
            raise RuntimeError(f"goal violates physical joint limits: {violated}")

        max_delta = float(np.max(np.abs(goal[:7] - start[:7])))
        samples = max(2, int(math.ceil(max_delta / 0.01)) + 1)
        check_data = mujoco.MjData(self.model)
        inactive = "left" if side == "right" else "right"
        inactive_model = physical_to_model_position(
            inactive, physical[inactive], self.model_version
        )
        self.resolver.set_qpos(check_data.qpos, inactive_model, inactive)

        for sample_index, alpha in enumerate(np.linspace(0.0, 1.0, samples)):
            waypoint = start + alpha * (goal - start)
            waypoint_model = physical_to_model_position(
                side, waypoint, self.model_version
            )
            self.resolver.set_qpos(check_data.qpos, waypoint_model, side)
            mujoco.mj_forward(self.model, check_data)
            for contact_index in range(check_data.ncon):
                contact = check_data.contact[contact_index]
                geom1 = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
                ) or f"geom{contact.geom1}"
                geom2 = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
                ) or f"geom{contact.geom2}"
                if not (f"openarm_{side}_" in geom1 or f"openarm_{side}_" in geom2):
                    continue
                # Closed v1 gripper fingers intentionally contact each other.
                if (
                    f"openarm_{side}_" in geom1
                    and f"openarm_{side}_" in geom2
                    and "finger_collision" in geom1
                    and "finger_collision" in geom2
                ):
                    continue
                progress = 100.0 * sample_index / (samples - 1)
                raise RuntimeError(
                    f"predicted collision at {progress:.1f}%: {geom1} <-> {geom2}"
                )
        if verbose:
            print(
                f"[hardware] path check passed: {samples} samples, "
                f"max joint travel={max_delta:.3f} rad",
                flush=True,
            )

    def solve_once(self) -> None:
        if not self.follow:
            return
        # Mouse input updates the desired target immediately, but IK receives a
        # rate-limited target.  This prevents a large cursor event from causing
        # one rejected joint jump and permanently stalling the arm.
        position_delta = self.target[:3] - self.ik_target[:3]
        position_distance = float(np.linalg.norm(position_delta))
        if position_distance > 0.0:
            self.ik_target[:3] += position_delta * min(
                1.0, self.target_position_step / position_distance
            )
        self.ik_target[3:] = approach_quat(
            self.ik_target[3:], self.target[3:], self.target_orientation_step
        )
        previous_right, previous_left = self._drivers()
        previous = np.r_[previous_right, previous_left]
        inactive = "left" if self.side == "right" else "right"
        inactive_pose = self._fk(inactive)
        orientation_cost = (
            self.strict_orientation_cost
            if time.monotonic() < self.orientation_strict_until
            else TRANSLATION_ORIENTATION_COST
        )
        self.kin.set_orientation_cost(self.side, orientation_cost)
        self.kin.set_orientation_cost(inactive, ROTATION_ORIENTATION_COST)
        self.kin.set_target(self.side, self.ik_target.astype(np.float32))
        self.kin.set_target(inactive, inactive_pose.astype(np.float32))
        result = self.kin.solve()
        if result is None or result.shape != (16,) or not np.all(np.isfinite(result)):
            self.reachable = False
            detail = getattr(self.kin._ik, "last_failure", "")
            self.last_reject_reason = detail or "IK returned no finite solution"
            return
        # Simulation safety filter: reject discontinuous IK branches.
        active_slice = slice(0, 8) if self.side == "right" else slice(8, 16)
        joint_step = float(np.max(np.abs(result[active_slice] - previous[active_slice])))
        if joint_step > 0.06:
            self.reachable = False
            self.last_reject_reason = f"joint step rejected: {joint_step:.3f} rad"
            self._sync_from_model()
            return
        active = result[:8] if self.side == "right" else result[8:]
        self.resolver.set_qpos(self.data.qpos, active, self.side)
        mujoco.mj_forward(self.model, self.data)
        actual = self._fk(self.side)
        self.position_error = float(np.linalg.norm(actual[:3] - self.ik_target[:3]))
        self.orientation_error = quat_error_deg(actual[3:], self.ik_target[3:])
        self.reachable = self.position_error < 0.015 and self.orientation_error < 5.0
        self.last_reject_reason = "" if self.reachable else "IK tracking error too large"
        now = time.monotonic()
        if now - self.last_log_time >= 1.0:
            queued = float(np.linalg.norm(self.target[:3] - self.ik_target[:3]))
            reason = self.last_reject_reason or "none"
            print(
                f"[drag] side={self.side} reachable={self.reachable} "
                f"queued={queued * 1000:.1f}mm pos_err={self.position_error * 1000:.2f}mm "
                f"rot_err={self.orientation_error:.2f}deg max_dq={joint_step:.4f} "
                f"reject={reason}",
                flush=True,
            )
            self.last_log_time = now

    def draw(self) -> None:
        width, height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.option,
            None,
            self.camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            self.scene,
        )
        inactive = "left" if self.side == "right" else "right"
        add_target_frame(
            self.scene,
            self._fk(inactive),
            reachable=True,
            scale=0.05,
            active=False,
        )
        add_target_frame(self.scene, self.target, self.reachable, active=True)
        mujoco.mjr_render(viewport, self.scene, self.context)
        status = "OK" if self.reachable else "UNREACHABLE"
        run = "FOLLOW" if self.follow else "PAUSED"
        hardware_mode = "REAL" if self.hardware is not None else "SIM"
        armed = (
            f"ARMED-{self.hardware.armed_side.upper()}"
            if self.hardware is not None and self.hardware.armed_side is not None
            else "DISABLED"
        )
        moving = self.hardware is not None and self.hardware.move_goal is not None
        real_follow = self.hardware is not None and self.hardware.follow_enabled
        queued = float(np.linalg.norm(self.target[:3] - self.ik_target[:3]))
        help_left = (
            "TARGET\n"
            "Move in view plane\n"
            "Move horizontal / world Z\n"
            "Rotate target\n\n"
            "VIEW\n"
            "Orbit camera\n"
            "Pan camera\n"
            "Zoom\n\n"
            "KEYS\n"
            "Select left / right\n"
            "Switch arm\n"
            "Enable / disable real arm\n"
            "Toggle continuous real follow\n"
            "Move real arm to simulation\n"
            "Sync simulation to physical\n"
            "Reset target\n"
            "Pause / resume\n"
            "Quit"
        )
        help_right = (
            "\n"
            "Left drag\n"
            "Shift + Left drag\n"
            "Right drag\n\n"
            "\n"
            "Alt + Left drag\n"
            "Middle drag\n"
            "Mouse wheel\n\n"
            "\n"
            "1 / 2\n"
            "Tab\n"
            "E\n"
            "F\n"
            "D (press)\n"
            "P\n"
            "R\n"
            "Space\n"
            "Esc"
        )
        mujoco.mjr_overlay(
            mujoco.mjtFont.mjFONT_NORMAL,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            viewport,
            help_left,
            help_right,
            self.context,
        )
        glfw.set_window_title(
            self.window,
            f"OpenArm {self.model_version.upper()} {hardware_mode} | "
            f"{self.side.upper()} | {armed} | "
            f"REAL-FOLLOW={'ON' if real_follow else 'OFF'} | "
            f"MOVE={'RUNNING' if moving else 'IDLE'} | {run} | {status} | "
            f"queue {queued * 1000:.1f} mm | pos {self.position_error * 1000:.1f} mm | "
            f"rot {self.orientation_error:.1f} deg",
        )

    def run(self) -> None:
        try:
            while not glfw.window_should_close(self.window):
                self._apply_native_controls()
                self.solve_once()
                self.update_hardware()
                self._sync_native_controls()
                self.draw()
                glfw.swap_buffers(self.window)
                glfw.poll_events()
        finally:
            if self.hardware is not None:
                self.hardware.close()
            try:
                self.native_connection.send(("quit", None))
            except (BrokenPipeError, EOFError):
                pass
            self.native_process.join(timeout=2.0)
            if self.native_process.is_alive():
                self.native_process.terminate()
                self.native_process.join(timeout=1.0)
            self.native_connection.close()
            glfw.destroy_window(self.window)
            glfw.terminate()

    def update_hardware(self) -> None:
        if self.hardware is None:
            return
        if self.hardware.armed_side != self.side:
            return
        if not self.hardware.follow_enabled and self.hardware.move_goal is None:
            return
        # An unattended window must never continue sending commands.
        focused = (
            glfw.get_window_attrib(self.window, glfw.FOCUSED) == glfw.TRUE
            or time.monotonic() - self.last_native_activity < 1.0
        )
        if not focused:
            self.hardware.cancel_move("window lost focus")
            if self.hardware.follow_enabled:
                self.hardware.set_follow(False)
            return
        if self.hardware.follow_enabled:
            if not self.reachable:
                self.hardware.set_follow(False)
                print("[hardware] FOLLOW stopped: IK target is unreachable", flush=True)
                return
            if not self.hardware.command_due():
                return
            right, left = self._drivers()
            desired = right if self.side == "right" else left
            desired = desired.copy()
            if self.model_version == "v1":
                desired[7] = self.hardware.drivers[self.side].last_command[7]
            physical = {
                side: driver.last_command.copy()
                for side, driver in self.hardware.drivers.items()
            }
            next_command = limit_joint_command(
                physical[self.side], desired, self.hardware.max_step
            )
            try:
                self._validate_physical_path(
                    self.side, next_command, physical=physical, verbose=False
                )
                self.hardware.send_if_due(self.side, desired)
            except (RuntimeError, ValueError) as error:
                self.hardware.set_follow(False)
                print(f"[hardware] FOLLOW stopped: {error}", flush=True)
            return
        try:
            measured = self.hardware.send_if_due(self.side, self.hardware.move_goal)
        except Exception:
            self.hardware.disarm()
            raise
        if measured is not None:
            remaining = float(np.max(np.abs(measured - self.hardware.move_goal)))
            if remaining < 0.01:
                print(
                    f"[hardware] MOVE complete {self.side}; max error={remaining:.4f} rad",
                    flush=True,
                )
                self.hardware.move_goal = None


class NativeSingleWindowApp(DragApp):
    """Single stock MuJoCo viewer with mocap EE targets and Control sliders."""

    def __init__(
        self, side: str, model_version: str, hardware: HardwareBridge | None = None
    ) -> None:
        if model_version != "v1":
            raise RuntimeError("single-window native control currently supports v1")
        self.hardware = hardware
        self.model_version = model_version
        xml = PROJECT_ROOT / "models/openarm_v1/scene_native_control.xml"
        base_spec = MODEL_SPECS["v1"]
        self.setup = ArmSetup.from_args(
            xml=str(xml),
            mode="bimanual",
            frame_right=base_spec["frames"]["right"],
            frame_type_right=base_spec["frame_type"],
            frame_left=base_spec["frames"]["left"],
            frame_type_left=base_spec["frame_type"],
            keyframe=None,
        )
        self.kin = SafeKinematics(
            self.setup,
            IKParams(
                max_iters=15,
                dt=0.08,
                damping=0.1,
                posture_cost=0.03,
                orientation_cost=TRANSLATION_ORIENTATION_COST,
                lm_damping=0.01,
            ),
        )
        self.model = self.setup.model
        self.data = self.setup.data
        self.resolver = self.setup.joint_resolver
        self.side = side
        self.follow = True
        self.reachable = True
        self.position_error = 0.0
        self.orientation_error = 0.0
        self.last_log_time = 0.0
        self.last_reject_reason = ""
        self.orientation_strict_until = 0.0
        self.target_position_step = 0.0025
        self.target_orientation_step = math.radians(1.0)
        self.strict_orientation_cost = ROTATION_ORIENTATION_COST
        self.pending_keys: list[int] = []
        self.close_requested = False
        self.last_auto_reset = 0.0

        if self.hardware is not None:
            for arm_side in ("right", "left"):
                self.resolver.set_qpos(
                    self.data.qpos,
                    physical_to_model_position(
                        arm_side,
                        self.hardware.initial_positions[arm_side],
                        model_version,
                    ),
                    arm_side,
                )
        mujoco.mj_forward(self.model, self.data)
        self._sync_from_model()
        self.target = self._fk(self.side).copy()
        self.ik_target = self.target.copy()

        self.mocap_ids = {}
        self.target_geom_ids = {}
        for arm_side in ("right", "left"):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{arm_side}_ee_target"
            )
            self.mocap_ids[arm_side] = int(self.model.body_mocapid[body_id])
            self.target_geom_ids[arm_side] = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                f"{arm_side}_ee_target_geom",
            )
            self._set_mocap(arm_side, self._fk(arm_side))
        self._update_target_colors()

        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            if (
                joint_id >= 0
                and self.model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE
            ):
                self.model.actuator_ctrllimited[actuator_id] = 1
                self.model.actuator_ctrlrange[actuator_id] = self.model.jnt_range[
                    joint_id
                ]
                self.data.ctrl[actuator_id] = self.data.qpos[
                    int(self.model.jnt_qposadr[joint_id])
                ]
        self.native_ctrl_snapshot = self.data.ctrl.copy()

    def _set_mocap(self, side: str, pose: np.ndarray) -> None:
        mocap_id = self.mocap_ids[side]
        self.data.mocap_pos[mocap_id] = pose[:3]
        self.data.mocap_quat[mocap_id] = pose[3:]

    def _mocap_pose(self, side: str) -> np.ndarray:
        mocap_id = self.mocap_ids[side]
        return np.r_[self.data.mocap_pos[mocap_id], self.data.mocap_quat[mocap_id]]

    def _update_target_colors(self) -> None:
        for arm_side, geom_id in self.target_geom_ids.items():
            if arm_side == self.side:
                self.model.geom_size[geom_id, 0] = 0.025
                self.model.geom_rgba[geom_id] = [0.1, 0.9, 0.25, 0.8]
            else:
                self.model.geom_size[geom_id, 0] = 0.018
                self.model.geom_rgba[geom_id] = [0.4, 0.7, 1.0, 0.4]

    def _native_key_callback(self, key: int) -> None:
        self.pending_keys.append(key)

    def _handle_native_keys(self, viewer: object) -> None:
        while self.pending_keys:
            key = self.pending_keys.pop(0)
            if key in (ord("1"), ord("2")):
                self._select_side("left" if key == ord("1") else "right")
                self.target = self._mocap_pose(self.side)
                self.ik_target = self.target.copy()
                self._update_target_colors()
            elif key == glfw.KEY_TAB:
                self._select_side("left" if self.side == "right" else "right")
                self.target = self._mocap_pose(self.side)
                self.ik_target = self.target.copy()
                self._update_target_colors()
            elif key in (ord(" "), glfw.KEY_SPACE):
                self.follow = not self.follow
            elif key in (ord("R"), ord("r")):
                pose = self._fk(self.side)
                self._set_mocap(self.side, pose)
                self.target = pose.copy()
                self.ik_target = pose.copy()
                self.reachable = True
            elif key in (ord("E"), ord("e")) and self.hardware is not None:
                if self.hardware.armed_side == self.side:
                    self.hardware.disarm()
                else:
                    self.hardware.arm(self.side)
            elif key in (ord("F"), ord("f")) and self.hardware is not None:
                if self.hardware.armed_side != self.side:
                    print("[hardware] F ignored: select a side and press E first", flush=True)
                else:
                    self.hardware.set_follow(not self.hardware.follow_enabled)
            elif key in (ord("D"), ord("d")) and self.hardware is not None:
                self._start_native_move()
            elif key in (ord("P"), ord("p")) and self.hardware is not None:
                self._sync_simulation_to_physical()
                for arm_side in ("right", "left"):
                    self._set_mocap(arm_side, self._fk(arm_side))
            elif key == glfw.KEY_ESCAPE:
                self.close_requested = True

    def _start_native_move(self) -> None:
        assert self.hardware is not None
        if self.hardware.armed_side != self.side:
            print("[hardware] D ignored: select a side and press E first", flush=True)
            return
        if not self.reachable:
            print("[hardware] D ignored: target is not reachable", flush=True)
            return
        right, left = self._drivers()
        goal = (right if self.side == "right" else left).copy()
        goal[7] = self.hardware.drivers[self.side].last_command[7]
        try:
            self._validate_physical_path(self.side, goal)
            self.hardware.set_move_goal(self.side, goal)
        except (RuntimeError, ValueError) as error:
            print(f"[hardware] D ignored: {error}", flush=True)

    def _apply_control_sliders(self) -> None:
        changed = np.flatnonzero(
            np.abs(self.data.ctrl - self.native_ctrl_snapshot) > 1e-7
        )
        changed_sides: set[str] = set()
        for actuator_id in changed:
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            if (
                joint_id < 0
                or self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE
            ):
                continue
            self.data.qpos[int(self.model.jnt_qposadr[joint_id])] = self.data.ctrl[
                actuator_id
            ]
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
            ) or ""
            changed_sides.add("right" if "right" in name else "left")
        if changed_sides:
            mujoco.mj_forward(self.model, self.data)
            self._sync_from_model()
            for arm_side in changed_sides:
                self._set_mocap(arm_side, self._fk(arm_side))
            self.target = self._mocap_pose(self.side)
            self.ik_target = self.target.copy()
            self.reachable = True

    def _sync_control_sliders(self) -> None:
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            if (
                joint_id >= 0
                and self.model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE
            ):
                self.data.ctrl[actuator_id] = self.data.qpos[
                    int(self.model.jnt_qposadr[joint_id])
                ]
        self.native_ctrl_snapshot = self.data.ctrl.copy()

    def _reset_unreachable_follow_target(self) -> None:
        """Keep F active but return an unreachable mocap target to current FK."""
        pose = self._fk(self.side)
        self._sync_from_model()
        self._set_mocap(self.side, pose)
        self.target = pose.copy()
        self.ik_target = pose.copy()
        self.reachable = True
        self.position_error = 0.0
        self.orientation_error = 0.0
        self.last_reject_reason = ""
        now = time.monotonic()
        if now - self.last_auto_reset >= 0.5:
            print(
                f"[hardware] FOLLOW target unreachable; auto-R reset to "
                f"current {self.side} endpoint (follow remains ON)",
                flush=True,
            )
            self.last_auto_reset = now

    def update_hardware(self) -> None:
        if self.hardware is None or self.hardware.armed_side != self.side:
            return
        if not self.hardware.follow_enabled and self.hardware.move_goal is None:
            return
        if self.hardware.follow_enabled:
            if not self.reachable:
                self.hardware.set_follow(False)
                print("[hardware] FOLLOW stopped: IK target is unreachable", flush=True)
                return
            if not self.hardware.command_due():
                return
            right, left = self._drivers()
            desired = (right if self.side == "right" else left).copy()
            desired[7] = self.hardware.drivers[self.side].last_command[7]
            physical = {
                arm_side: driver.last_command.copy()
                for arm_side, driver in self.hardware.drivers.items()
            }
            next_command = limit_joint_command(
                physical[self.side], desired, self.hardware.max_step
            )
            try:
                self._validate_physical_path(
                    self.side, next_command, physical=physical, verbose=False
                )
                self.hardware.send_if_due(self.side, desired)
            except (RuntimeError, ValueError) as error:
                self.hardware.set_follow(False)
                print(f"[hardware] FOLLOW stopped: {error}", flush=True)
            return
        measured = self.hardware.send_if_due(self.side, self.hardware.move_goal)
        if measured is not None:
            remaining = float(np.max(np.abs(measured - self.hardware.move_goal)))
            if remaining < 0.01:
                print(f"[hardware] MOVE complete {self.side}", flush=True)
                self.hardware.move_goal = None

    def run(self) -> None:
        try:
            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                key_callback=self._native_key_callback,
                show_left_ui=True,
                show_right_ui=True,
            ) as viewer:
                viewer.cam.azimuth = 135.0
                viewer.cam.elevation = -18.0
                viewer.cam.distance = 1.7
                while viewer.is_running():
                    start = time.monotonic()
                    with viewer.lock():
                        self._handle_native_keys(viewer)
                        self._apply_control_sliders()
                        self.target = self._mocap_pose(self.side)
                        self.solve_once()
                        if (
                            not self.reachable
                            and self.hardware is not None
                            and self.hardware.follow_enabled
                        ):
                            self._reset_unreachable_follow_target()
                        self._sync_control_sliders()
                    if self.close_requested:
                        viewer.close()
                        continue
                    self.update_hardware()
                    viewer.sync()
                    time.sleep(max(0.0, 1.0 / 60.0 - (time.monotonic() - start)))
        finally:
            if self.hardware is not None:
                self.hardware.close()


def headless_test(side: str, model_version: str) -> int:
    """Exercise the same IK path without opening a window."""
    spec = MODEL_SPECS[model_version]
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
            orientation_cost=1.0,
            lm_damping=0.01,
        ),
    )
    resolver = setup.joint_resolver
    r7, rg = resolver.get_driver(setup.data.qpos, "right")
    l7, lg = resolver.get_driver(setup.data.qpos, "left")
    right, left = np.r_[r7, rg].astype(np.float32), np.r_[l7, lg].astype(np.float32)
    kin.sync(np.r_[right, left])
    initial = np.asarray(kin.fk(side, right if side == "right" else left))
    inactive = "left" if side == "right" else "right"
    inactive_pose = np.asarray(kin.fk(inactive, left if side == "right" else right))
    target = initial.copy()
    target[:3] += (
        np.array([0.04, 0.0, 0.0])
        if model_version == "v1"
        else np.array([0.04, 0.0, 0.05])
    )
    result = None
    previous = np.r_[right, left]
    max_step = 0.0
    for alpha in np.linspace(0.0, 1.0, 101)[1:]:
        waypoint = initial.copy()
        waypoint[:3] += alpha * (target[:3] - initial[:3])
        kin.set_target(side, waypoint.astype(np.float32))
        kin.set_target(inactive, inactive_pose.astype(np.float32))
        result = kin.solve()
        if result is None or not np.all(np.isfinite(result)):
            raise RuntimeError(f"IK failed at path sample {alpha:.2f}")
        max_step = max(max_step, float(np.max(np.abs(result - previous))))
        previous = result.copy()
    assert result is not None
    active = result[:8] if side == "right" else result[8:]
    actual = np.asarray(kin.fk(side, active))
    pos_error = float(np.linalg.norm(actual[:3] - target[:3]))
    rot_error = quat_error_deg(actual[3:], target[3:])
    print(f"{model_version} {side} target: {np.array2string(target, precision=5)}")
    print(f"{side} actual: {np.array2string(actual, precision=5)}")
    print(f"position error: {pos_error * 1000:.3f} mm")
    print(f"orientation error: {rot_error:.4f} deg")
    print(f"maximum joint step: {max_step:.6f} rad")
    if pos_error > 0.005 or rot_error > 1.0 or max_step > 0.03:
        raise RuntimeError("headless drag-path validation failed")
    previous_command = np.zeros(8)
    desired_command = np.array([1.0, -1.0, 0.2, -0.2, 0.01, -0.01, 0.0, 0.5])
    limited_command = limit_joint_command(previous_command, desired_command, 0.003)
    if float(np.max(np.abs(limited_command - previous_command))) > 0.003000001:
        raise RuntimeError("physical command rate limiter failed")
    print("SUCCESS: interactive controller IK path passed (simulation only).")
    print("SUCCESS: physical command rate limiter passed (no CAN access).")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument(
        "--model-version",
        choices=tuple(MODEL_SPECS),
        default="v1",
        help="OpenArm hardware/model generation (default: v1)",
    )
    parser.add_argument(
        "--headless-test",
        action="store_true",
        help="validate a scripted 4 cm forward / 5 cm upward drag without a window",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="connect to the physical arms (requires --confirm-hardware)",
    )
    parser.add_argument(
        "--confirm-hardware",
        action="store_true",
        help="explicit confirmation that the workspace is clear and E-stop is accessible",
    )
    parser.add_argument(
        "--config",
        type=pathlib.Path,
        default=PROJECT_ROOT / "config/openarm_safe_current.yaml",
        help="physical driver configuration",
    )
    parser.add_argument("--command-hz", type=float, default=20.0)
    parser.add_argument(
        "--max-joint-step",
        type=float,
        default=0.003,
        help="maximum physical joint change per command [rad]",
    )
    args = parser.parse_args()
    if args.headless_test:
        return headless_test(args.side, args.model_version)
    hardware = None
    if args.real:
        if not args.confirm_hardware:
            parser.error("--real requires --confirm-hardware")
        if args.command_hz <= 0.0 or not 0.0 < args.max_joint_step <= 0.01:
            parser.error("real mode requires command-hz > 0 and 0 < max-joint-step <= 0.01")
        print(
            "REAL MODE: reading both arms; no motor is enabled until E is pressed. "
            "Press D once to move slowly to the current simulation pose.",
            flush=True,
        )
        hardware = HardwareBridge(args.config, args.command_hz, args.max_joint_step)
    NativeSingleWindowApp(args.side, args.model_version, hardware=hardware).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
