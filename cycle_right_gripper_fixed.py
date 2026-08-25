#!/usr/bin/env python3
"""Safely cycle right J8 with correct OpenArm-CAN initialization ordering."""

import math
import time

import openarm_can as oa


OPEN_TRAVEL = math.pi / 3.0
STEP = 0.002
HZ = 20.0
KP = 20.0
KD = 0.8
MAX_TORQUE = 1.0
MAX_ERROR = 0.15


def main() -> int:
    robot = oa.OpenArm("can0", True)
    # Match the official `openarm-can-cli monitor --id 8` path: a one-motor
    # ArmComponent avoids GripperComponent's parameter-write initialization.
    robot.init_arm_motors([oa.MotorType.DM4310], [0x08], [0x18])
    component = robot.get_arm()
    enabled = False

    def state():
        # get_motors() returns Motor value snapshots, not live references.
        # Fetch a new snapshot after every recv_all(); caching it freezes state.
        motor = component.get_motors()[0]
        values = (
            float(motor.get_position()), float(motor.get_velocity()),
            float(motor.get_torque()), int(motor.get_state_tmos()),
            int(motor.get_state_trotor()),
        )
        if not all(math.isfinite(value) for value in values[:3]):
            raise RuntimeError(f"invalid J8 state: {values}")
        return values

    def command(target):
        component.mit_control_one(0, oa.MITParam(KP, KD, target, 0.0, 0.0))
        robot.recv_all(20000)
        time.sleep(1.0 / HZ)
        return state()

    def move(start, target, label):
        commanded = start
        peak = 0.0
        while abs(target - commanded) > STEP:
            commanded += math.copysign(STEP, target - commanded)
            q, velocity, torque, _, _ = command(commanded)
            peak = max(peak, abs(torque))
            if abs(torque) > MAX_TORQUE:
                raise RuntimeError(f"{label}: torque {torque:+.3f} Nm")
            if abs(q - commanded) > MAX_ERROR:
                raise RuntimeError(
                    f"{label}: tracking q={q:+.3f}, command={commanded:+.3f}"
                )
        commanded = target
        for _ in range(20):
            q, velocity, torque, _, _ = command(commanded)
            peak = max(peak, abs(torque))
        error = q - target
        if abs(error) > 0.02:
            raise RuntimeError(f"{label}: final error {error:+.3f} rad")
        print(
            f"{label}: q={q:+.6f}, velocity={velocity:+.4f}, "
            f"error={error:+.6f}, peak_torque={peak:.3f} Nm",
            flush=True,
        )
        return commanded

    try:
        robot.set_callback_mode_all(oa.CallbackMode.STATE)

        # Follow the official CLI order exactly: enable, wait, receive, then
        # refresh, wait, and receive before trusting the state object.
        component.enable_all()
        enabled = True
        time.sleep(0.10)
        robot.recv_all(50000)
        component.refresh_all()
        time.sleep(0.02)
        robot.recv_all(50000)
        q, velocity, torque, tmos, trotor = state()
        if tmos <= 0 or trotor <= 0:
            raise RuntimeError(
                f"missing live enable feedback: q={q}, temp={tmos}/{trotor}"
            )
        print(
            f"TAKEOVER: q={q:+.6f}, velocity={velocity:+.4f}, "
            f"torque={torque:+.3f}, temp={tmos}/{trotor} C",
            flush=True,
        )

        commanded = q
        for _ in range(20):
            current, _, torque, _, _ = command(commanded)
            if abs(current - commanded) > 0.03 or abs(torque) > MAX_TORQUE:
                raise RuntimeError("current-position takeover hold failed")

        # Prove feedback and direction before committing to the full travel.
        small_target = commanded - 0.03
        commanded = move(commanded, small_target, "SMALL OPEN")
        commanded = move(commanded, q, "SMALL RETURN")

        open_target = q - OPEN_TRAVEL
        commanded = move(commanded, open_target, "MAXIMUM OPEN")
        move(commanded, q, "FULLY CLOSED")
        print("SUCCESS: right J8 completed one maximum-travel open/close cycle.")
        return 0
    finally:
        if enabled:
            component.disable_all()
            robot.recv_all(20000)
        print("Safety cleanup: right J8 disabled; J1-J7 never enabled.")


if __name__ == "__main__":
    raise SystemExit(main())
