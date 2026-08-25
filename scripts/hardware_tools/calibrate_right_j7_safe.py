#!/usr/bin/env python3
"""Safely calibrate only OpenArm right-arm J7 (CAN ID 7).

Default mode finds the positive mechanical stop and exits with J7 disabled,
without writing zero. With --home-and-write-zero, the script finds the stop,
moves back exactly 90 degrees, disables J7, and stores that position as zero.
J1-J6 and the gripper are never initialized or enabled.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import openarm_can as oa


SEND_ID = 0x07
RECV_ID = 0x17
ZERO_OFFSET_FROM_POSITIVE_STOP_RAD = math.radians(-90.0)


def read_state(motor: oa.Motor) -> tuple[float, float, float]:
    state = (
        float(motor.get_position()),
        float(motor.get_velocity()),
        float(motor.get_torque()),
    )
    if not all(math.isfinite(value) for value in state):
        raise RuntimeError(f"invalid feedback: {state}")
    return state


def refresh_stable(robot, joint) -> float:
    samples = []
    for _ in range(8):
        joint.refresh_all()
        robot.recv_all()
        time.sleep(0.01)
        q, _, _ = read_state(joint.get_motors()[0])
        samples.append(q)
    if max(samples[-5:]) - min(samples[-5:]) > math.radians(1.0):
        raise RuntimeError(f"unstable initial feedback: {samples[-5:]}")
    print(f"Stable position samples: {samples[-5:]}")
    return samples[-1]


def command_and_read(robot, joint, q_target, kp, kd):
    joint.mit_control_one(0, oa.MITParam(kp, kd, q_target, 0.0, 0.0))
    robot.recv_all()
    time.sleep(0.01)
    return read_state(joint.get_motors()[0])


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calibrate only right-arm J7 (ID 7); all other motors untouched."
    )
    parser.add_argument("--canport", default="can0")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--home-and-write-zero", action="store_true")
    parser.add_argument("--max-travel-deg", type=float, default=185.0)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--step-deg", type=float, default=0.05)
    parser.add_argument("--kp", type=float, default=15.0)
    parser.add_argument("--kd", type=float, default=0.8)
    parser.add_argument("--stop-torque", type=float, default=1.0)
    parser.add_argument("--max-torque", type=float, default=3.0)
    parser.add_argument("--stop-velocity", type=float, default=0.10)
    parser.add_argument("--confirm-samples", type=int, default=5)
    parser.add_argument("--max-following-error-deg", type=float, default=10.0)
    args = parser.parse_args()

    if args.max_travel_deg <= 0 or args.timeout <= 0 or args.step_deg <= 0:
        parser.error("travel, timeout, and step must be positive")
    if args.confirm_samples < 2:
        parser.error("confirm-samples must be at least 2")
    if args.stop_torque >= args.max_torque:
        parser.error("stop-torque must be below max-torque")

    print(f"CAN interface : {args.canport}")
    print(f"Motor         : right J7 only, ID {SEND_ID} -> {RECV_ID}")
    print(f"Mode          : {'HOME + WRITE ZERO' if args.home_and_write_zero else 'FIND STOP ONLY'}")
    print(f"Travel limit  : {args.max_travel_deg:.1f} deg")
    print("J1-J6 and gripper are not initialized and cannot be enabled.")

    robot = oa.OpenArm(args.canport, True)
    robot.init_arm_motors([oa.MotorType.DM4310], [SEND_ID], [RECV_ID])
    robot.set_callback_mode_all(oa.CallbackMode.STATE)
    joint = robot.get_arm()
    enabled = False
    zero_written = False

    try:
        q_start = refresh_stable(robot, joint)
        print(f"Initial J7 position: {q_start:.6f} rad ({math.degrees(q_start):.2f} deg)")
        if args.check_only:
            print("CHECK PASSED: J7 was not enabled or moved.")
            return 0

        joint.enable_all()
        enabled = True
        time.sleep(0.1)

        # Verify that enabling and holding the measured pose cannot cause a jump.
        last_q = q_start
        for _ in range(20):
            q, _, torque = command_and_read(robot, joint, q_start, args.kp, args.kd)
            if abs(q - q_start) > math.radians(2.0):
                raise RuntimeError(
                    f"moved during hold check: {q_start:.6f} -> {q:.6f} rad"
                )
            if abs(torque) > args.max_torque:
                raise RuntimeError(f"hold torque exceeded limit: {torque:.3f} Nm")
            last_q = q

        step = math.radians(args.step_deg)
        max_travel = math.radians(args.max_travel_deg)
        max_following_error = math.radians(args.max_following_error_deg)
        q_target = q_start
        started = time.monotonic()
        stop_hits = 0
        q_stop = None

        while True:
            elapsed = time.monotonic() - started
            actual_travel = last_q - q_start
            if elapsed >= args.timeout:
                raise RuntimeError(f"timeout after {elapsed:.2f} s")
            if actual_travel > max_travel:
                raise RuntimeError(
                    f"maximum actual travel reached ({math.degrees(actual_travel):.2f} deg)"
                )
            # Advance only while the real joint is following. At a mechanical
            # stop the target is allowed to lead by at most the configured
            # following error, which builds detectable torque without an
            # unbounded position command.
            if q_target - last_q < max_following_error:
                q_target += step  # Right J7 positive direction -> +90 deg stop.
            q, velocity, torque = command_and_read(
                robot, joint, q_target, args.kp, args.kd
            )
            if q < last_q - math.radians(1.0):
                raise RuntimeError(
                    f"unexpected reverse motion: {last_q:.6f} -> {q:.6f} rad"
                )
            if abs(q - last_q) > math.radians(5.0):
                raise RuntimeError(f"feedback jump: {last_q:.6f} -> {q:.6f} rad")
            if q - q_start > max_travel + math.radians(1.0):
                raise RuntimeError("actual travel exceeded limit")
            last_q = q
            if abs(torque) > args.max_torque:
                raise RuntimeError(f"torque exceeded limit: {torque:.3f} Nm")

            if abs(velocity) < args.stop_velocity and abs(torque) > args.stop_torque:
                stop_hits += 1
            else:
                stop_hits = 0
            if stop_hits >= args.confirm_samples:
                q_stop = q
                print(
                    "Positive stop confirmed: "
                    f"q={q:.6f} rad ({math.degrees(q):.2f} deg), "
                    f"velocity={velocity:.4f} rad/s, torque={torque:.3f} Nm, "
                    f"actual travel={math.degrees(q - q_start):.2f} deg, "
                    f"following error={math.degrees(q_target - q):.2f} deg"
                )
                break

        if not args.home_and_write_zero:
            joint.disable_all()
            enabled = False
            robot.recv_all()
            print("TEST PASSED: stop found; J7 disabled; zero was NOT changed.")
            return 0

        # Move from +90-degree mechanical stop back to the mechanical center.
        q_goal = q_stop + ZERO_OFFSET_FROM_POSITIVE_STOP_RAD
        print(f"Moving 90 deg away from stop to zero candidate: {q_goal:.6f} rad")
        return_step = math.radians(0.10)
        q_command = q_stop
        return_started = time.monotonic()
        last_q = q_stop
        while q_command - q_goal > 1e-6:
            if time.monotonic() - return_started > 15.0:
                raise RuntimeError("return-to-zero timeout")
            q_command = max(q_goal, q_command - return_step)
            q, _, torque = command_and_read(robot, joint, q_command, args.kp, args.kd)
            if q > last_q + math.radians(1.0):
                raise RuntimeError(f"wrong direction while returning: {last_q} -> {q}")
            if abs(q - last_q) > math.radians(5.0):
                raise RuntimeError(f"feedback jump while returning: {last_q} -> {q}")
            if abs(torque) > args.max_torque:
                raise RuntimeError(f"return torque exceeded limit: {torque:.3f} Nm")
            last_q = q

        joint.disable_all()
        enabled = False
        robot.recv_all()
        joint.set_zero_all()
        robot.recv_all()
        zero_written = True
        print("SUCCESS: right J7 mechanical center stored as zero.")
        return 0

    except KeyboardInterrupt:
        print("\nSTOPPED: Ctrl+C; zero was not written.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ABORTED: {exc}; zero was not written.", file=sys.stderr)
        return 1
    finally:
        try:
            if enabled or not zero_written:
                joint.disable_all()
                robot.recv_all()
                print("Safety cleanup: J7 disabled.")
        except Exception as exc:
            print(f"WARNING: failed to confirm J7 disable: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
