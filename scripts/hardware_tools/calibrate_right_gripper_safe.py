#!/usr/bin/env python3
"""Safely home only the OpenArm right gripper (CAN ID 8).

The script initializes and enables only the gripper component. Arm joints
J1-J7 are never initialized, enabled, or commanded. By default it only finds
the closed mechanical stop and exits without changing the stored zero. Pass
--write-zero explicitly to store the detected closed position as zero.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import openarm_can as oa


SEND_ID = 0x08
RECV_ID = 0x18


def finite_state(motor: oa.Motor) -> tuple[float, float, float]:
    values = (
        float(motor.get_position()),
        float(motor.get_velocity()),
        float(motor.get_torque()),
    )
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"invalid motor feedback: {values}")
    return values


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Home only the right gripper (ID 8); J1-J7 are untouched."
    )
    parser.add_argument("--canport", default="can0")
    parser.add_argument(
        "--write-zero",
        action="store_true",
        help="store the detected closed stop as zero (default: do not write)",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="refresh and print feedback without enabling or moving the gripper",
    )
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--max-travel-deg", type=float, default=65.0)
    parser.add_argument("--step-deg", type=float, default=0.1)
    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.8)
    parser.add_argument("--torque-threshold", type=float, default=0.30)
    parser.add_argument("--max-torque", type=float, default=1.00)
    parser.add_argument("--velocity-threshold", type=float, default=0.30)
    parser.add_argument("--confirm-samples", type=int, default=5)
    args = parser.parse_args()

    if args.timeout <= 0 or args.max_travel_deg <= 0 or args.step_deg <= 0:
        parser.error("timeout, max travel, and step must be positive")
    if args.confirm_samples < 2:
        parser.error("confirm-samples must be at least 2")

    print(f"CAN interface : {args.canport}")
    print(f"Motor         : right gripper only, ID {SEND_ID} -> {RECV_ID}")
    print(f"Maximum travel: {args.max_travel_deg:.1f} deg toward closed stop")
    print(f"Timeout       : {args.timeout:.1f} s")
    print(f"Write zero    : {'YES' if args.write_zero else 'NO (test only)'}")
    print("J1-J7 are not initialized and will not be enabled.")

    robot = oa.OpenArm(args.canport, True)
    robot.init_gripper_motor(oa.MotorType.DM4310, SEND_ID, RECV_ID)
    robot.set_callback_mode_all(oa.CallbackMode.STATE)
    gripper = robot.get_gripper()

    enabled = False
    stop_found = False
    try:
        # Explicitly request state before enabling anything. recv_all() alone
        # may leave the motor object at its decoder's placeholder position.
        position_samples = []
        for _ in range(8):
            gripper.refresh_all()
            robot.recv_all()
            time.sleep(0.01)
            motor = gripper.get_motors()[0]
            q, _, _ = finite_state(motor)
            position_samples.append(q)
        q_start = position_samples[-1]
        if max(position_samples[-5:]) - min(position_samples[-5:]) > 0.05:
            raise RuntimeError(
                f"unstable initial feedback: {position_samples[-5:]}"
            )
        print(f"Initial position: {q_start:.6f} rad")

        if args.check_only:
            print(f"CHECK PASSED: stable samples={position_samples[-5:]}")
            print("Gripper was not enabled; no motion or zero write was requested.")
            return 0

        gripper.enable_all()
        enabled = True
        time.sleep(0.10)

        # First command the freshly measured position and verify that enabling
        # cannot produce a large jump before beginning the closing sweep.
        last_q = q_start
        for _ in range(20):
            gripper.mit_control_one(
                0, oa.MITParam(args.kp, args.kd, q_start, 0.0, 0.0)
            )
            robot.recv_all()
            time.sleep(0.01)
            motor = gripper.get_motors()[0]
            q, _, torque = finite_state(motor)
            if abs(q - q_start) > math.radians(2.0):
                raise RuntimeError(
                    f"position moved during hold check: {q_start:.6f} -> {q:.6f} rad"
                )
            if abs(torque) > args.max_torque:
                raise RuntimeError(
                    f"torque safety limit exceeded during hold: {torque:.3f} Nm"
                )
            last_q = q

        step_rad = math.radians(args.step_deg)
        max_travel_rad = math.radians(args.max_travel_deg)
        q_target = q_start
        started = time.monotonic()
        consecutive_hits = 0

        while True:
            elapsed = time.monotonic() - started
            traveled = q_target - q_start
            if elapsed >= args.timeout:
                raise RuntimeError(f"timeout after {elapsed:.2f} s")
            if traveled + step_rad > max_travel_rad:
                raise RuntimeError(
                    f"maximum travel reached ({math.degrees(traveled):.2f} deg)"
                )

            q_target += step_rad  # Positive direction closes the right gripper.
            gripper.mit_control_one(
                0, oa.MITParam(args.kp, args.kd, q_target, 0.0, 0.0)
            )
            robot.recv_all()
            time.sleep(0.005)

            motor = gripper.get_motors()[0]
            q, velocity, torque = finite_state(motor)
            if q < last_q - math.radians(1.0):
                raise RuntimeError(
                    f"unexpected opening motion: {last_q:.6f} -> {q:.6f} rad"
                )
            if abs(q - last_q) > math.radians(5.0):
                raise RuntimeError(
                    f"feedback jump: {last_q:.6f} -> {q:.6f} rad"
                )
            if q - q_start > max_travel_rad + math.radians(1.0):
                raise RuntimeError("actual position exceeded maximum travel")
            last_q = q
            if abs(torque) > args.max_torque:
                raise RuntimeError(
                    f"torque safety limit exceeded: {torque:.3f} Nm "
                    f"> {args.max_torque:.3f} Nm"
                )
            if (
                abs(velocity) < args.velocity_threshold
                and abs(torque) > args.torque_threshold
            ):
                consecutive_hits += 1
            else:
                consecutive_hits = 0

            if consecutive_hits >= args.confirm_samples:
                stop_found = True
                print(
                    "Closed stop confirmed: "
                    f"position={q:.6f} rad, velocity={velocity:.4f} rad/s, "
                    f"torque={torque:.3f} Nm, "
                    f"commanded travel={math.degrees(q_target - q_start):.2f} deg"
                )
                break

        # Remove torque before changing persistent calibration data.
        gripper.disable_all()
        enabled = False
        robot.recv_all()
        print("Gripper disabled.")

        if args.write_zero:
            gripper.set_zero()
            robot.recv_all()
            print("SUCCESS: right gripper closed stop stored as zero.")
        else:
            print("TEST PASSED: stop found; zero was NOT changed.")
            print("Run again with --write-zero only after reviewing this result.")
        return 0

    except KeyboardInterrupt:
        print("\nSTOPPED: Ctrl+C received; zero was not written.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ABORTED: {exc}; zero was not written.", file=sys.stderr)
        return 1
    finally:
        if enabled or not stop_found:
            try:
                gripper.disable_all()
                robot.recv_all()
                print("Safety cleanup: gripper disabled.")
            except Exception as exc:
                print(
                    f"WARNING: disable command could not be confirmed: {exc}",
                    file=sys.stderr,
                )


if __name__ == "__main__":
    raise SystemExit(main())
