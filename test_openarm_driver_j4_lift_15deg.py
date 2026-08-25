#!/usr/bin/env python3
"""Lift the physical right arm by moving only J4 +5 degrees, then return."""

import math
import time

import numpy as np
from openarm_driver import Config, SingleArmDriver


DELTA = math.radians(5.0)
MAX_TORQUES = np.array([10.5, 10.5, 4.0, 12.0, 2.0, 2.0, 2.0, 2.0])


def main() -> int:
    driver = SingleArmDriver("right_arm", Config("openarm_safe_current.yaml"))
    started = False
    try:
        samples = []
        for _ in range(10):
            samples.append(driver.fetch_position(refresh=True))
            time.sleep(0.03)
        samples = np.asarray(samples)
        q0 = samples[-1].copy()
        if not np.all(np.isfinite(samples)):
            raise RuntimeError("non-finite initial feedback")
        if np.max(np.ptp(samples[-5:], axis=0)) > 0.01:
            raise RuntimeError("unstable initial feedback")

        target = q0.copy()
        target[3] += DELTA
        print("Initial qpos:", np.array2string(q0, precision=5), flush=True)
        print("J4 command: +5.00 deg", flush=True)
        print("Target qpos:", np.array2string(target, precision=5), flush=True)

        driver.start()
        started = True
        previous = q0.copy()
        peak = np.zeros(8)

        def command(qcmd: np.ndarray) -> None:
            nonlocal previous, peak
            driver.send_position(qcmd)
            state = driver.fetch_state(refresh=True)
            q, torque = state["qpos"], state["qtorque"]
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(torque)):
                raise RuntimeError("non-finite motor state")
            if np.any(np.abs(q - previous) > 0.05):
                raise RuntimeError(f"feedback jump: {q - previous}")
            if np.any(np.abs(qcmd - q) > 0.18):
                raise RuntimeError(f"following error: {qcmd - q}")
            if np.any(np.abs(torque) > MAX_TORQUES):
                j = int(np.argmax(np.abs(torque) / MAX_TORQUES))
                raise RuntimeError(
                    f"J{j + 1} torque {torque[j]:.3f} Nm exceeds "
                    f"{MAX_TORQUES[j]:.1f} Nm"
                )
            previous = q.copy()
            peak = np.maximum(peak, np.abs(torque))
            time.sleep(0.02)

        # Take over without a jump.
        for _ in range(50):
            command(q0)

        # Six-second lift.
        for alpha in np.linspace(0.0, 1.0, 301)[1:]:
            command(q0 + alpha * (target - q0))
        reached = previous.copy()
        print(
            f"Reached J4: {math.degrees(reached[3] - q0[3]):+.2f} deg",
            flush=True,
        )

        # Hold raised for two seconds.
        for _ in range(100):
            command(target)

        # Six-second controlled return.
        for alpha in np.linspace(1.0, 0.0, 301)[1:]:
            command(q0 + alpha * (target - q0))
        for _ in range(50):
            command(q0)

        error = previous - q0
        print("Return error:", np.array2string(error, precision=5), flush=True)
        print("Peak torque:", np.array2string(peak, precision=3), flush=True)
        if np.any(np.abs(error) > 0.025):
            raise RuntimeError(f"return error exceeds 0.025 rad: {error}")
        print("SUCCESS: J4 lift completed and returned.", flush=True)
        return 0
    finally:
        if started:
            driver.stop()
        else:
            try:
                driver.openarm.disable_all()
            except Exception:
                pass
        print("Safety cleanup: right arm disabled.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
