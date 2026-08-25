#!/usr/bin/env python3
"""Verify that SingleArmDriver can take over and hold the measured pose."""

import argparse
import time

import numpy as np
from openarm_driver import Config, SingleArmDriver


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=["right", "left"], required=True)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()
    name = f"{args.side}_arm"
    driver = SingleArmDriver(name, Config("openarm_safe_current.yaml"))
    started = False
    try:
        q0 = driver.fetch_position(refresh=True)
        if q0.shape != (8,) or not np.all(np.isfinite(q0)):
            raise RuntimeError(f"invalid initial position: {q0}")
        print("Initial driver qpos:", np.array2string(q0, precision=5))
        driver.start()
        started = True
        # Immediately hold the position measured before enabling.
        deadline = time.monotonic() + args.duration
        peak_torque = np.zeros(8)
        while time.monotonic() < deadline:
            driver.send_position(q0)
            state = driver.fetch_state(refresh=True)
            peak_torque = np.maximum(peak_torque, np.abs(state["qtorque"]))
            if not all(np.all(np.isfinite(state[key])) for key in ("qpos", "qvel", "qtorque")):
                raise RuntimeError("non-finite driver state")
            time.sleep(0.02)
        final = driver.fetch_position(refresh=True)
        error = final - q0
        print("Final driver qpos:", np.array2string(final, precision=5))
        print("Hold errors      :", np.array2string(error, precision=5))
        print("Peak torques (Nm):", np.array2string(peak_torque, precision=3))
        if np.any(np.abs(error) > 0.03):
            raise RuntimeError(f"hold error exceeds 0.03 rad: {error}")
        print(f"SUCCESS: {args.side} driver current-pose hold passed.")
        return 0
    finally:
        if started:
            driver.stop()
        else:
            try:
                driver.openarm.disable_all()
            except Exception:
                pass
        print(f"Safety cleanup: {args.side} arm disabled.")


if __name__ == "__main__":
    raise SystemExit(main())
