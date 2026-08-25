#!/usr/bin/env python3
"""Small, guarded +0.01 rad round-trip test for right-arm J7 only."""

import math
import sys
import time

import openarm_can as oa


DELTA = 0.03
KP = 20.0
KD = 0.8
MAX_TORQUE = 2.0
MAX_FOLLOWING_ERROR = 0.03
MAX_FEEDBACK_STEP = 0.02


def state(joint):
    m = joint.get_motors()[0]
    values = (float(m.get_position()), float(m.get_velocity()), float(m.get_torque()))
    if not all(math.isfinite(v) for v in values):
        raise RuntimeError(f"invalid feedback: {values}")
    return values


def command(robot, joint, target, previous_q):
    joint.mit_control_one(0, oa.MITParam(KP, KD, target, 0.0, 0.0))
    robot.recv_all()
    time.sleep(0.01)
    q, velocity, torque = state(joint)
    if abs(q - previous_q) > MAX_FEEDBACK_STEP:
        raise RuntimeError(f"feedback jump: {previous_q:.6f} -> {q:.6f} rad")
    if abs(target - q) > MAX_FOLLOWING_ERROR:
        raise RuntimeError(f"following error too large: target={target:.6f}, q={q:.6f}")
    if abs(torque) > MAX_TORQUE:
        raise RuntimeError(f"torque limit exceeded: {torque:.3f} Nm")
    return q, velocity, torque


def main():
    robot = oa.OpenArm("can0", True)
    robot.init_arm_motors([oa.MotorType.DM4310], [0x07], [0x17])
    robot.set_callback_mode_all(oa.CallbackMode.STATE)
    joint = robot.get_arm()
    enabled = False
    try:
        samples = []
        for _ in range(10):
            joint.refresh_all()
            robot.recv_all()
            time.sleep(0.02)
            samples.append(state(joint)[0])
        if max(samples[-5:]) - min(samples[-5:]) > 0.01:
            raise RuntimeError(f"unstable initial feedback: {samples[-5:]}")
        q0 = samples[-1]
        print(f"Initial J7: {q0:+.6f} rad ({math.degrees(q0):+.3f} deg)")

        joint.enable_all()
        enabled = True
        time.sleep(0.1)
        q = q0

        # Hold the measured pose before moving.
        for _ in range(30):
            q, _, _ = command(robot, joint, q0, q)
        if abs(q - q0) > 0.01:
            raise RuntimeError("J7 moved during initial hold")

        # Ramp +0.01 rad over one second.
        peak_q = q
        for i in range(1, 101):
            target = q0 + DELTA * i / 100.0
            q, velocity, torque = command(robot, joint, target, q)
            peak_q = max(peak_q, q)
        if peak_q < q0 + 0.003:
            raise RuntimeError("J7 did not move in the commanded positive direction")
        print(
            f"Positive step reached: q={q:+.6f} rad, "
            f"delta={q-q0:+.6f} rad, torque={torque:+.3f} Nm"
        )

        for _ in range(50):
            q, velocity, torque = command(robot, joint, q0 + DELTA, q)

        # Ramp back to the exact starting position over one second.
        for i in range(1, 101):
            target = q0 + DELTA * (1.0 - i / 100.0)
            q, velocity, torque = command(robot, joint, target, q)
        for _ in range(30):
            q, velocity, torque = command(robot, joint, q0, q)

        print(
            f"Returned: q={q:+.6f} rad, error={q-q0:+.6f} rad, "
            f"velocity={velocity:+.4f} rad/s, torque={torque:+.3f} Nm"
        )
        if abs(q - q0) > 0.01:
            raise RuntimeError("return error exceeds 0.01 rad")
        print(f"SUCCESS: right J7 +{DELTA:.3f} rad round-trip completed.")
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
                joint.disable_all()
                robot.recv_all()
            print("Safety cleanup: J7 disabled.")
        except Exception as exc:
            print(f"WARNING: failed to confirm disable: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
