#!/usr/bin/env python3
"""Offline regression tests for official-driver physical VIVE commands."""

import importlib.metadata
import pathlib
import sys
import unittest
from collections import deque
from types import SimpleNamespace
from unittest import mock

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from openarm_driver import Config
from openarm_driver.safety import JointVelocityChecker
from openarm_teleop.interactive_mujoco_ee_drag import physical_to_model_position
from openarm_teleop.vive_mujoco_real_teleop import BimanualHardwareBridge


class OfficialDriverBridgeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.bridge = BimanualHardwareBridge.__new__(BimanualHardwareBridge)
        self.bridge.command_hz = 40.0
        self.bridge.command_period = 0.025
        self.bridge.last_command_time = {"left": 0.0}
        self.bridge.next_command_time = {"left": 0.0}
        self.bridge.command_intervals = {"left": deque(maxlen=120)}
        self.bridge.last_cadence_log = 100.0

    def test_project_uses_official_driver_0_3(self) -> None:
        self.assertEqual(importlib.metadata.version("openarm-driver"), "0.3.0")

    def test_official_velocity_checker_uses_elapsed_time(self) -> None:
        limits = np.array([2.0, 2.0, 3.3, 3.3, 6.3, 6.3, 6.3, 20.0])
        checker = JointVelocityChecker(limits)
        result = checker.check(
            np.ones(8),
            driver=SimpleNamespace(last_command=np.zeros(8)),
            dt_s=0.01,
        )
        self.assertFalse(result.is_safe)
        np.testing.assert_allclose(result.fixed_joint_positions, limits * 0.01)

    def test_machine_config_uses_official_motion_profile(self) -> None:
        config = Config(ROOT / "config/openarm_safe_raw_zero.yaml")
        official = Config("openarm_cell")
        for side in ("right_arm", "left_arm"):
            np.testing.assert_allclose(
                config.get_joint_limits(side)[:7],
                official.get_joint_limits(side)[:7],
            )
            np.testing.assert_allclose(
                config.get_joint_limits(side)[7], [-1.047198, 0.4]
            )
        np.testing.assert_allclose(
            config.get_joint_delta_position_limits(),
            [1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5, 3.14],
        )
        np.testing.assert_allclose(
            config.get_joint_velocity_limits(),
            [2.0, 2.0, 3.3, 3.3, 6.3, 6.3, 6.3, 20.0],
        )
        self.assertTrue(config.get_gripper_posforce())
        np.testing.assert_allclose(config.get_gripper_posforce_limits(), [5.0, 0.7])

    def test_both_installed_grippers_open_toward_negative_j8(self) -> None:
        for side in ("right", "left"):
            physical = np.zeros(8)
            physical[7] = -np.pi / 3.0
            model = physical_to_model_position(side, physical, "v1")
            self.assertAlmostEqual(model[7], 0.044)

    def test_command_deadline_keeps_phase_against_sixty_hz_ui(self) -> None:
        self.assertTrue(self.bridge.command_due("left", 100.0))
        self.bridge._mark_command_sent("left", 100.0)
        self.assertAlmostEqual(self.bridge.next_command_time["left"], 100.025)

        self.assertFalse(self.bridge.command_due("left", 100.017))
        self.assertTrue(self.bridge.command_due("left", 100.034))
        self.bridge._mark_command_sent("left", 100.034)
        self.assertAlmostEqual(self.bridge.next_command_time["left"], 100.05)
        self.assertTrue(self.bridge.command_due("left", 100.051))

    def test_bridge_passes_target_directly_to_official_driver(self) -> None:
        class FakeDriver:
            def __init__(self) -> None:
                self.last_command = np.zeros(8)
                self.latest_state = None
                self.received = None

            def send_position(self, position: np.ndarray) -> None:
                self.received = np.asarray(position).copy()
                self.last_command = self.received.copy()
                self.latest_state = {"qpos": self.last_command.copy()}

            def fetch_position(self, refresh: bool = True) -> np.ndarray:
                raise AssertionError("send_if_due must not issue a duplicate refresh")

        driver = FakeDriver()
        self.bridge.drivers = {"left": driver}
        self.bridge.armed = True
        target = np.array([0.4, -0.3, 0.2, 0.1, -0.2, 0.3, -0.4, 0.5])
        with mock.patch(
            "openarm_teleop.vive_mujoco_real_teleop.time.monotonic",
            side_effect=(100.0, 100.001),
        ):
            measured = self.bridge.send_if_due("left", target)

        np.testing.assert_allclose(driver.received, target)
        np.testing.assert_allclose(measured, target)
        self.assertAlmostEqual(self.bridge.next_command_time["left"], 100.026)


if __name__ == "__main__":
    unittest.main()
