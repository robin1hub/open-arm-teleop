#!/usr/bin/env python3
"""Guarded small synchronized pose test for the physical right arm."""

import math
import argparse
import sys
import time

import numpy as np
import openarm_can as oa


DELTAS = np.array([0.01, 0.01, 0.01, 0.01, 0.02, 0.02, 0.02, 0.0])
KPS = np.array([70.0, 70.0, 80.0, 150.0, 20.0, 20.0, 20.0, 20.0])
KDS = np.array([2.75, 2.5, 2.0, 2.0, 0.8, 0.8, 0.8, 0.8])
MAX_TORQUES = np.array([10.0, 10.0, 4.0, 8.0, 2.0, 2.0, 2.0, 2.0])


def states(arm, gripper):
    motors = list(arm.get_motors()) + list(gripper.get_motors())
    result = np.array(
        [[float(m.get_position()), float(m.get_velocity()), float(m.get_torque())] for m in motors]
    )
    if result.shape != (8, 3) or not np.all(np.isfinite(result)):
        raise RuntimeError(f"invalid state array: {result}")
    return result


def send(robot, arm, gripper, targets, previous_q):
    arm.mit_control_all(
        [oa.MITParam(float(KPS[i]), float(KDS[i]), float(targets[i]), 0.0, 0.0) for i in range(7)]
    )
    gripper.mit_control_one(
        0, oa.MITParam(float(KPS[7]), float(KDS[7]), float(targets[7]), 0.0, 0.0)
    )
    robot.recv_all()
    time.sleep(0.01)
    current = states(arm, gripper)
    q = current[:, 0]
    torque = current[:, 2]
    if np.any(np.abs(q - previous_q) > 0.05):
        raise RuntimeError(f"feedback jump: previous={previous_q}, current={q}")
    if np.any(np.abs(targets - q) > 0.12):
        raise RuntimeError(f"following error: target={targets}, current={q}")
    if np.any(np.abs(torque) > MAX_TORQUES):
        idx = int(np.argmax(np.abs(torque) / MAX_TORQUES))
        raise RuntimeError(
            f"J{idx+1} torque {torque[idx]:.3f} Nm exceeds {MAX_TORQUES[idx]:.3f} Nm"
        )
    return current


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--canport", default="can0")
    parser.add_argument("--side", choices=["right", "left"], default="right")
    args = parser.parse_args()
    robot = oa.OpenArm(args.canport, True)
    robot.init_arm_motors(
        [oa.MotorType.DM8009, oa.MotorType.DM8009, oa.MotorType.DM4340,
         oa.MotorType.DM4340, oa.MotorType.DM4310, oa.MotorType.DM4310,
         oa.MotorType.DM4310],
        [1, 2, 3, 4, 5, 6, 7],
        [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
    )
    robot.init_gripper_motor(oa.MotorType.DM4310, 8, 0x18)
    robot.set_callback_mode_all(oa.CallbackMode.STATE)
    arm, gripper = robot.get_arm(), robot.get_gripper()
    enabled = False
    try:
        samples = []
        for _ in range(10):
            arm.refresh_all(); gripper.refresh_all(); robot.recv_all(); time.sleep(0.02)
            samples.append(states(arm, gripper)[:, 0])
        q0 = samples[-1]
        if np.max(np.ptp(np.array(samples[-5:]), axis=0)) > 0.01:
            raise RuntimeError("unstable initial feedback")
        print("Initial positions:", np.array2string(q0, precision=5))
        print("Target deltas    :", np.array2string(DELTAS, precision=5))

        robot.enable_all()
        enabled = True
        time.sleep(0.1)
        current = states(arm, gripper)
        peak_torque = np.abs(current[:, 2])

        # Hold the exact measured pose for one second.
        for _ in range(100):
            current = send(robot, arm, gripper, q0, current[:, 0])
            peak_torque = np.maximum(peak_torque, np.abs(current[:, 2]))

        # Three-second ramp to the small synchronized target.
        for i in range(1, 301):
            target = q0 + DELTAS * (i / 300.0)
            current = send(robot, arm, gripper, target, current[:, 0])
            peak_torque = np.maximum(peak_torque, np.abs(current[:, 2]))
        reached = current[:, 0].copy()
        print("Reached deltas   :", np.array2string(reached - q0, precision=5))

        for _ in range(50):
            current = send(robot, arm, gripper, q0 + DELTAS, current[:, 0])
            peak_torque = np.maximum(peak_torque, np.abs(current[:, 2]))

        # Three-second return to the original measured pose.
        for i in range(1, 301):
            target = q0 + DELTAS * (1.0 - i / 300.0)
            current = send(robot, arm, gripper, target, current[:, 0])
            peak_torque = np.maximum(peak_torque, np.abs(current[:, 2]))
        for _ in range(100):
            current = send(robot, arm, gripper, q0, current[:, 0])
            peak_torque = np.maximum(peak_torque, np.abs(current[:, 2]))

        errors = current[:, 0] - q0
        print("Return errors    :", np.array2string(errors, precision=5))
        print("Peak torques (Nm):", np.array2string(peak_torque, precision=3))
        if np.any(np.abs(errors[:7]) > 0.02) or abs(errors[7]) > 0.02:
            raise RuntimeError(f"return error exceeds 0.02 rad: {errors}")
        print(f"SUCCESS: {args.side} whole-arm small synchronized pose completed.")
        return 0
    except KeyboardInterrupt:
        print("STOPPED by Ctrl+C", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ABORTED: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            if enabled:
                robot.disable_all(); robot.recv_all()
            print(f"Safety cleanup: all {args.side}-arm motors disabled.")
        except Exception as exc:
            print(f"WARNING: disable failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
