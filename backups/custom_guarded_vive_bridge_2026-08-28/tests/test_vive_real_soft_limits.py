#!/usr/bin/env python3
"""Offline regression tests for physical VIVE soft joint limits."""

import pathlib
import sys
import unittest
from collections import deque
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import numpy as np

from openarm_teleop.vive_mujoco_real_teleop import BimanualHardwareBridge


class SoftJointLimitTests(unittest.TestCase):
    def setUp(self) -> None:
        self.bridge = BimanualHardwareBridge.__new__(BimanualHardwareBridge)
        hard = np.array(
            [[-1.570796, 1.570796]] * 7 + [[-1.047198, 0.4]],
            dtype=np.float64,
        )
        soft = hard.copy()
        soft[:7, 0] += 0.05
        soft[:7, 1] -= 0.05
        self.bridge.joint_limits = {"left": hard}
        self.bridge.command_limits = {"left": soft}
        self.bridge.joint_velocity_limits = np.full(7, 1.0)
        self.bridge.max_command_step = 0.028
        self.bridge.gripper_velocity_limit = 0.04
        self.bridge.max_joint_acceleration = 1000.0
        self.bridge.last_command_velocity = {"left": np.zeros(8)}
        self.bridge.command_hz = 40.0
        self.bridge.command_period = 0.025
        self.bridge.last_command_time = {"left": 0.0}
        self.bridge.next_command_time = {"left": 0.0}
        self.bridge.command_intervals = {"left": deque(maxlen=120)}
        self.bridge.peak_tracking_error = {"left": 0.0}
        self.bridge.last_cadence_log = 100.0

    def test_exact_pi_over_two_is_projected_inside_yaml_limit(self) -> None:
        desired = np.zeros(8)
        desired[6] = np.pi / 2.0
        limited = self.bridge.clamp_to_command_limits("left", desired)
        self.assertAlmostEqual(limited[6], 1.570796 - 0.05)
        self.assertLess(limited[6], self.bridge.joint_limits["left"][6, 1])

    def test_values_inside_soft_range_are_unchanged(self) -> None:
        desired = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, 0.1])
        limited = self.bridge.clamp_to_command_limits("left", desired)
        np.testing.assert_allclose(limited, desired)

    def test_gripper_keeps_hard_range_without_arm_margin(self) -> None:
        desired = np.zeros(8)
        desired[7] = 1.0
        limited = self.bridge.clamp_to_command_limits("left", desired)
        self.assertAlmostEqual(limited[7], 0.4)

    def test_gripper_has_independent_slower_rate_limit(self) -> None:
        previous = np.zeros(8)
        desired = np.ones(8)
        limited = self.bridge.limit_command("left", previous, desired)
        np.testing.assert_allclose(limited[:7], 0.025)
        self.assertAlmostEqual(limited[7], 0.001)

    def test_arm_velocity_limits_are_per_joint_and_time_based(self) -> None:
        self.bridge.joint_velocity_limits = np.arange(1.0, 8.0)
        self.bridge.max_joint_acceleration = 10000.0
        previous = np.zeros(8)
        desired = np.ones(8)
        limited = self.bridge.limit_command("left", previous, desired, dt_s=0.002)
        np.testing.assert_allclose(
            limited[:7], 0.002 * self.bridge.joint_velocity_limits
        )

    def test_single_command_guard_still_caps_a_delayed_loop(self) -> None:
        self.bridge.joint_velocity_limits = np.full(7, 10.0)
        previous = np.zeros(8)
        desired = np.ones(8)
        limited = self.bridge.limit_command("left", previous, desired, dt_s=0.1)
        np.testing.assert_allclose(limited[:7], 0.028)

    def test_arm_acceleration_ramps_without_overshooting(self) -> None:
        self.bridge.max_joint_acceleration = 2.0
        previous = np.zeros(8)
        desired = np.ones(8)
        first = self.bridge.limit_command("left", previous, desired, dt_s=0.1)
        np.testing.assert_allclose(first[:7], 0.02)
        self.bridge.last_command_velocity["left"] = (first - previous) / 0.1
        second = self.bridge.limit_command("left", first, desired, dt_s=0.1)
        np.testing.assert_allclose(second[:7] - first[:7], 0.028)

        self.bridge.last_command_velocity["left"][:7] = 0.4
        close_target = second.copy()
        close_target[:7] += 0.0002
        final = self.bridge.limit_command("left", second, close_target, dt_s=0.1)
        np.testing.assert_allclose(final[:7], close_target[:7])

    def test_arm_reversal_brakes_before_changing_direction(self) -> None:
        self.bridge.max_joint_acceleration = 2.0
        self.bridge.last_command_velocity["left"][:7] = 0.4
        previous = np.zeros(8)
        desired = -np.ones(8)
        limited = self.bridge.limit_command("left", previous, desired, dt_s=0.1)
        np.testing.assert_allclose(limited[:7], 0.0)

    def test_command_deadline_keeps_phase_against_sixty_hz_ui(self) -> None:
        self.assertTrue(self.bridge.command_due("left", 100.0))
        self.bridge._mark_command_sent("left", 100.0)
        self.assertAlmostEqual(self.bridge.next_command_time["left"], 100.025)

        self.assertFalse(self.bridge.command_due("left", 100.017))
        self.assertTrue(self.bridge.command_due("left", 100.034))
        self.bridge._mark_command_sent("left", 100.034)
        self.assertAlmostEqual(self.bridge.next_command_time["left"], 100.05)

        # The following UI frame is allowed to send. Resetting the deadline
        # from 100.034 instead would incorrectly wait until roughly 100.068.
        self.assertTrue(self.bridge.command_due("left", 100.051))

    def test_send_uses_feedback_already_received_by_driver(self) -> None:
        class FakeDriver:
            def __init__(self) -> None:
                self.last_command = np.zeros(8)
                self.latest_state = None

            def send_position(self, position: np.ndarray) -> None:
                self.last_command = np.asarray(position).copy()
                self.latest_state = {"qpos": self.last_command.copy()}

            def fetch_position(self, refresh: bool = True) -> np.ndarray:
                raise AssertionError("send_if_due must not issue a duplicate refresh")

        driver = FakeDriver()
        self.bridge.drivers = {"left": driver}
        self.bridge.armed = True
        self.bridge.max_tracking_error = 0.15
        validated: list[np.ndarray] = []
        with mock.patch(
            "openarm_teleop.vive_mujoco_real_teleop.time.monotonic",
            side_effect=(100.0, 100.001),
        ):
            measured = self.bridge.send_if_due(
                "left", np.full(8, 0.001), validator=validated.append
            )

        np.testing.assert_allclose(measured, np.full(8, 0.001))
        np.testing.assert_allclose(validated, [np.full(8, 0.001)])
        self.assertAlmostEqual(self.bridge.next_command_time["left"], 100.026)


if __name__ == "__main__":
    unittest.main()
