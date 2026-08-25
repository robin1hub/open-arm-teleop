#!/usr/bin/env python3
"""Guarded 5-degree wrist motion through the official OpenArm driver."""

import math
import time

import numpy as np
from openarm_driver import Config, SingleArmDriver


DELTAS = np.array([0.0, 0.0, 0.0, 0.0, math.radians(5), -math.radians(5), math.radians(5), 0.0])
MAX_TORQUES = np.array([10.0, 10.0, 4.0, 8.0, 2.0, 2.0, 2.0, 2.0])


def main() -> int:
    driver = SingleArmDriver("right_arm", Config("openarm_safe_current.yaml"))
    started = False
    try:
        samples = []
        for _ in range(10):
            samples.append(driver.fetch_position(refresh=True))
            time.sleep(0.02)
        q0 = samples[-1].copy()
        if not np.all(np.isfinite(samples)):
            raise RuntimeError("non-finite initial feedback")
        if np.max(np.ptp(np.asarray(samples[-5:]), axis=0)) > 0.01:
            raise RuntimeError("unstable initial feedback")
        target = q0 + DELTAS
        print("Initial qpos :", np.array2string(q0, precision=5))
        print("Delta degrees:", np.array2string(np.degrees(DELTAS), precision=2))
        print("Target qpos  :", np.array2string(target, precision=5))

        driver.start()
        started = True
        peak = np.zeros(8)
        previous = q0.copy()

        def command(qcmd: np.ndarray) -> np.ndarray:
            nonlocal previous, peak
            driver.send_position(qcmd)
            state = driver.fetch_state(refresh=True)
            q = state["qpos"]
            torque = state["qtorque"]
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(torque)):
                raise RuntimeError("non-finite motor state")
            if np.any(np.abs(q - previous) > 0.05):
                raise RuntimeError(f"feedback jump: {q - previous}")
            if np.any(np.abs(qcmd - q) > 0.15):
                raise RuntimeError(f"following error: {qcmd - q}")
            if np.any(np.abs(torque) > MAX_TORQUES):
                j = int(np.argmax(np.abs(torque) / MAX_TORQUES))
                raise RuntimeError(f"J{j + 1} torque {torque[j]:.3f} Nm exceeds limit")
            previous = q.copy()
            peak = np.maximum(peak, np.abs(torque))
            time.sleep(0.02)
            return q

        for _ in range(50):
            command(q0)
        for alpha in np.linspace(0.0, 1.0, 201)[1:]:
            command(q0 + alpha * DELTAS)
        reached = previous.copy()
        print("Reached deg  :", np.array2string(np.degrees(reached - q0), precision=2))
        for _ in range(50):
            command(target)
        for alpha in np.linspace(1.0, 0.0, 201)[1:]:
            command(q0 + alpha * DELTAS)
        for _ in range(50):
            command(q0)

        error = previous - q0
        print("Return error :", np.array2string(error, precision=5))
        print("Peak torque  :", np.array2string(peak, precision=3))
        if np.any(np.abs(error) > 0.025):
            raise RuntimeError(f"return error exceeds 0.025 rad: {error}")
        print("SUCCESS: OpenArm driver wrist motion completed and returned.")
        return 0
    finally:
        if started:
            driver.stop()
        else:
            try:
                driver.openarm.disable_all()
            except Exception:
                pass
        print("Safety cleanup: right arm disabled.")


if __name__ == "__main__":
    raise SystemExit(main())
