#!/usr/bin/env python3
"""Guardedly return only left-arm J7 (can1 ID 7) to stored zero."""

import math
import sys
import time

import openarm_can as oa


def read_state(joint):
    motor = joint.get_motors()[0]
    values = (
        float(motor.get_position()),
        float(motor.get_velocity()),
        float(motor.get_torque()),
    )
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"invalid feedback: {values}")
    return values


def main():
    robot = oa.OpenArm("can1", True)
    robot.init_arm_motors([oa.MotorType.DM4310], [0x07], [0x17])
    robot.set_callback_mode_all(oa.CallbackMode.STATE)
    joint = robot.get_arm()
    enabled = False
    try:
        samples = []
        for _ in range(10):
            joint.refresh_all(); robot.recv_all(); time.sleep(0.02)
            samples.append(read_state(joint)[0])
        if max(samples[-5:]) - min(samples[-5:]) > 0.01:
            raise RuntimeError(f"unstable initial feedback: {samples[-5:]}")
        q0 = samples[-1]
        print(f"Initial left J7: {q0:+.6f} rad ({math.degrees(q0):+.2f} deg)")
        if abs(q0) > 0.30:
            raise RuntimeError("initial J7 is more than 0.30 rad from zero")

        joint.enable_all(); enabled = True; time.sleep(0.1)
        q_previous = q0
        # Hold current pose before beginning the return.
        for _ in range(30):
            joint.mit_control_one(0, oa.MITParam(20.0, 0.8, q0, 0.0, 0.0))
            robot.recv_all(); time.sleep(0.01)
            q, _, torque = read_state(joint)
            if abs(q - q_previous) > 0.03 or abs(torque) > 2.0:
                raise RuntimeError("hold safety limit exceeded")
            q_previous = q

        # Five-second linear move from the measured position to stored zero.
        for i in range(1, 501):
            target = q0 * (1.0 - i / 500.0)
            joint.mit_control_one(0, oa.MITParam(20.0, 0.8, target, 0.0, 0.0))
            robot.recv_all(); time.sleep(0.01)
            q, velocity, torque = read_state(joint)
            if abs(q - q_previous) > 0.03:
                raise RuntimeError(f"feedback jump: {q_previous:.6f} -> {q:.6f}")
            if q < q_previous - 0.01:
                raise RuntimeError(f"unexpected negative motion: {q_previous:.6f} -> {q:.6f}")
            if abs(target - q) > 0.08:
                raise RuntimeError(f"following error: target={target:.6f}, q={q:.6f}")
            if abs(torque) > 2.0:
                raise RuntimeError(f"torque limit exceeded: {torque:.3f} Nm")
            q_previous = q

        for _ in range(100):
            joint.mit_control_one(0, oa.MITParam(20.0, 0.8, 0.0, 0.0, 0.0))
            robot.recv_all(); time.sleep(0.01)
            q, velocity, torque = read_state(joint)
            if abs(torque) > 2.0:
                raise RuntimeError(f"zero hold torque exceeded: {torque:.3f} Nm")
        print(
            f"Reached left J7: {q:+.6f} rad ({math.degrees(q):+.3f} deg), "
            f"velocity={velocity:+.4f}, torque={torque:+.3f} Nm"
        )
        if abs(q) > 0.02:
            raise RuntimeError(f"final error exceeds 0.02 rad: {q:.6f}")
        print("SUCCESS: left J7 returned to stored zero.")
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
                joint.disable_all(); robot.recv_all()
            print("Safety cleanup: left J7 disabled.")
        except Exception as exc:
            print(f"WARNING: disable failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
