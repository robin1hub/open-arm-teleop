#!/usr/bin/env python3
"""Sequential guarded round-trip test for the remaining right-arm joints."""

import math
import argparse
import sys
import time

import openarm_can as oa


TESTS = [
    # id, name, motor type, delta(rad), kp, kd, max torque
    (7, "J7", oa.MotorType.DM4310, +0.03, 20.0, 0.8, 2.0),
    (6, "J6", oa.MotorType.DM4310, +0.03, 20.0, 0.8, 2.0),
    (5, "J5", oa.MotorType.DM4310, +0.03, 20.0, 0.8, 2.0),
    (4, "J4", oa.MotorType.DM4340, +0.030, 150.0, 2.0, 8.0),
    (3, "J3", oa.MotorType.DM4340, +0.03, 80.0, 1.5, 4.0),
    (2, "J2", oa.MotorType.DM8009, +0.03, 70.0, 2.0, 10.0),
    (1, "J1", oa.MotorType.DM8009, +0.03, 70.0, 2.0, 10.0),
    (8, "J8/gripper", oa.MotorType.DM4310, -0.03, 20.0, 0.8, 2.0),
]


def read_state(component):
    m = component.get_motors()[0]
    values = (float(m.get_position()), float(m.get_velocity()), float(m.get_torque()))
    if not all(math.isfinite(v) for v in values):
        raise RuntimeError(f"invalid feedback: {values}")
    return values


def command(robot, component, target, previous_q, kp, kd, max_torque):
    component.mit_control_one(0, oa.MITParam(kp, kd, target, 0.0, 0.0))
    robot.recv_all()
    time.sleep(0.01)
    q, velocity, torque = read_state(component)
    if abs(q - previous_q) > 0.03:
        raise RuntimeError(f"feedback jump: {previous_q:.6f} -> {q:.6f}")
    if abs(target - q) > 0.06:
        raise RuntimeError(f"following error: target={target:.6f}, q={q:.6f}")
    if abs(torque) > max_torque:
        raise RuntimeError(f"torque {torque:.3f} Nm exceeds {max_torque:.3f} Nm")
    return q, velocity, torque


def run_one(motor_id, name, motor_type, delta, kp, kd, max_torque, canport):
    robot = oa.OpenArm(canport, True)
    recv_id = motor_id + 0x10
    if motor_id == 8:
        robot.init_gripper_motor(motor_type, motor_id, recv_id)
        component = robot.get_gripper()
    else:
        robot.init_arm_motors([motor_type], [motor_id], [recv_id])
        component = robot.get_arm()
    robot.set_callback_mode_all(oa.CallbackMode.STATE)
    enabled = False
    try:
        samples = []
        for _ in range(10):
            component.refresh_all()
            robot.recv_all()
            time.sleep(0.02)
            samples.append(read_state(component)[0])
        if max(samples[-5:]) - min(samples[-5:]) > 0.01:
            raise RuntimeError(f"unstable initial feedback: {samples[-5:]}")
        q0 = samples[-1]
        print(f"[{name}] initial={q0:+.6f} rad, delta={delta:+.3f} rad")

        component.enable_all()
        enabled = True
        time.sleep(0.1)
        q = q0
        for _ in range(30):
            q, _, _ = command(robot, component, q0, q, kp, kd, max_torque)
        if abs(q - q0) > 0.01:
            raise RuntimeError("moved during initial hold")

        peak_change = 0.0
        for i in range(1, 101):
            target = q0 + delta * i / 100.0
            q, velocity, torque = command(
                robot, component, target, q, kp, kd, max_torque
            )
            peak_change = max(peak_change, abs(q - q0))
        if peak_change < min(0.003, abs(delta) * 0.25):
            raise RuntimeError("insufficient motion in commanded direction")
        if (q - q0) * delta <= 0:
            raise RuntimeError("motion direction disagrees with command")
        reached_q = q
        reached_torque = torque

        for _ in range(30):
            q, velocity, torque = command(
                robot, component, q0 + delta, q, kp, kd, max_torque
            )
        return_target = 0.0 if motor_id == 8 else q0
        ramp_start = q0 + delta
        for i in range(1, 101):
            target = ramp_start + (return_target - ramp_start) * i / 100.0
            q, velocity, torque = command(
                robot, component, target, q, kp, kd, max_torque
            )
        final_hold_samples = 100 if motor_id in (1, 2, 7, 8) else 30
        for _ in range(final_hold_samples):
            q, velocity, torque = command(
                robot, component, return_target, q, kp, kd, max_torque
            )
        error = q - return_target
        if abs(error) > 0.01:
            raise RuntimeError(f"return error {error:.6f} rad exceeds 0.01 rad")
        print(
            f"[{name}] PASS moved={reached_q-q0:+.6f} rad, "
            f"peak_torque={reached_torque:+.3f} Nm, "
            f"return_error={error:+.6f} rad"
        )
        return {
            "name": name,
            "moved": reached_q - q0,
            "torque": reached_torque,
            "error": error,
        }
    finally:
        try:
            if enabled:
                component.disable_all()
                robot.recv_all()
            print(f"[{name}] disabled")
        except Exception as exc:
            print(f"[{name}] WARNING disable failed: {exc}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ids",
        default="7,6,5,4,3,2,1,8",
        help="comma-separated motor IDs to test in the listed sequence",
    )
    parser.add_argument("--canport", default="can0")
    parser.add_argument("--side", choices=["right", "left"], default="right")
    args = parser.parse_args()
    requested = [int(value) for value in args.ids.split(",")]
    tests_by_id = {test[0]: test for test in TESTS}
    if any(motor_id not in tests_by_id for motor_id in requested):
        parser.error("IDs must be selected from 1,2,3,4,5,6,7,8")
    results = []
    try:
        for motor_id in requested:
            test = tests_by_id[motor_id]
            results.append(run_one(*test, args.canport))
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("STOPPED by Ctrl+C", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"BATCH ABORTED: {exc}", file=sys.stderr)
        return 1

    print(f"\n{args.side.upper()} ARM SEQUENTIAL TEST COMPLETE")
    for result in results:
        print(
            f"{result['name']}: moved={result['moved']:+.6f} rad, "
            f"torque={result['torque']:+.3f} Nm, "
            f"return_error={result['error']:+.6f} rad"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
