#!/usr/bin/env python3
"""Offline regression tests for physical VIVE soft joint limits."""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from vive_mujoco_real_teleop import BimanualHardwareBridge


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
        self.bridge.max_step = 0.004
        self.bridge.max_gripper_step = 0.001
        self.bridge.max_step_acceleration = 1.0
        self.bridge.last_command_delta = {"left": np.zeros(8)}

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
        np.testing.assert_allclose(limited[:7], 0.004)
        self.assertAlmostEqual(limited[7], 0.001)

    def test_arm_acceleration_ramps_without_overshooting(self) -> None:
        self.bridge.max_step_acceleration = 0.001
        previous = np.zeros(8)
        desired = np.ones(8)
        first = self.bridge.limit_command("left", previous, desired)
        np.testing.assert_allclose(first[:7], 0.001)
        self.bridge.last_command_delta["left"] = first - previous
        second = self.bridge.limit_command("left", first, desired)
        np.testing.assert_allclose(second[:7] - first[:7], 0.002)

        self.bridge.last_command_delta["left"][:7] = 0.004
        close_target = second.copy()
        close_target[:7] += 0.0002
        final = self.bridge.limit_command("left", second, close_target)
        np.testing.assert_allclose(final[:7], close_target[:7])

    def test_arm_reversal_brakes_before_changing_direction(self) -> None:
        self.bridge.max_step_acceleration = 0.001
        self.bridge.last_command_delta["left"][:7] = 0.004
        previous = np.zeros(8)
        desired = -np.ones(8)
        limited = self.bridge.limit_command("left", previous, desired)
        np.testing.assert_allclose(limited[:7], 0.0)


if __name__ == "__main__":
    unittest.main()
