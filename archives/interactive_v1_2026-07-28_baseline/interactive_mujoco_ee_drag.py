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
import pathlib
import time

import glfw
import mujoco
import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_control import ArmSetup, IKParams
from safe_kinematics import SafeKinematics


MODEL_SPECS = {
    "v1": {
        "xml": pathlib.Path(__file__).resolve().parent
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
        driver.start()
        # start() refreshes last_command from measured qpos and start.moves is
        # empty in the safe config, so enabling cannot initiate a trajectory.
        self.armed_side = side
        self.move_goal = None
        self.last_command_time = 0.0
        print(f"[hardware] ARMED {side}; press D to move to simulation pose", flush=True)

    def disarm(self) -> None:
        if self.armed_side is None:
            return
        side = self.armed_side
        self.armed_side = None
        self.move_goal = None
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

    def close(self) -> None:
        self.disarm()
        for driver in self.drivers.values():
            driver.openarm.disable_all()


def limit_joint_command(previous: np.ndarray, desired: np.ndarray, max_step: float) -> np.ndarray:
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
                    current = self.hardware.drivers[self.side].last_command
                    delta = np.abs(goal[:7] - current[:7])
                    if float(np.max(delta)) > 0.35 or float(np.linalg.norm(delta)) > 0.7:
                        raise RuntimeError(
                            "simulation pose is too far from the physical arm; "
                            "use a smaller staged move"
                        )
                    self.hardware.set_move_goal(self.side, goal)
                except (RuntimeError, ValueError) as error:
                    print(f"[hardware] D ignored: {error}", flush=True)
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

    def solve_once(self) -> None:
        if not self.follow:
            return
        # Mouse input updates the desired target immediately, but IK receives a
        # rate-limited target.  This prevents a large cursor event from causing
        # one rejected joint jump and permanently stalling the arm.
        position_delta = self.target[:3] - self.ik_target[:3]
        position_distance = float(np.linalg.norm(position_delta))
        if position_distance > 0.0:
            self.ik_target[:3] += position_delta * min(1.0, 0.0025 / position_distance)
        self.ik_target[3:] = approach_quat(
            self.ik_target[3:], self.target[3:], math.radians(1.0)
        )
        previous_right, previous_left = self._drivers()
        previous = np.r_[previous_right, previous_left]
        inactive = "left" if self.side == "right" else "right"
        inactive_pose = self._fk(inactive)
        orientation_cost = (
            ROTATION_ORIENTATION_COST
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
            "Move real arm to simulation\n"
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
            "D (press)\n"
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
            f"MOVE={'RUNNING' if moving else 'IDLE'} | {run} | {status} | "
            f"queue {queued * 1000:.1f} mm | pos {self.position_error * 1000:.1f} mm | "
            f"rot {self.orientation_error:.1f} deg",
        )

    def run(self) -> None:
        try:
            while not glfw.window_should_close(self.window):
                self.solve_once()
                self.update_hardware()
                self.draw()
                glfw.swap_buffers(self.window)
                glfw.poll_events()
        finally:
            if self.hardware is not None:
                self.hardware.close()
            glfw.destroy_window(self.window)
            glfw.terminate()

    def update_hardware(self) -> None:
        if self.hardware is None:
            return
        if self.hardware.armed_side != self.side or self.hardware.move_goal is None:
            return
        # An unattended window must never continue an autonomous catch-up move.
        focused = glfw.get_window_attrib(self.window, glfw.FOCUSED) == glfw.TRUE
        if not focused:
            self.hardware.cancel_move("window lost focus")
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
        default=pathlib.Path("openarm_safe_current.yaml"),
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
    DragApp(args.side, args.model_version, hardware=hardware).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
