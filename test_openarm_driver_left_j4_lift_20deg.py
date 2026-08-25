#!/usr/bin/env python3
"""Guarded left-arm J4 +30 degree lift with torque logging."""

import math
import time

import numpy as np
from openarm_driver import Config, SingleArmDriver


DELTA = math.radians(30.0)
MAX_TORQUES = np.array([10.0, 10.0, 4.0, 10.5, 2.0, 2.0, 2.0, 2.0])


def main() -> int:
    driver = SingleArmDriver("left_arm", Config("openarm_safe_current.yaml"))
    started = False
    try:
        samples = []
        for _ in range(12):
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
        print(f"Initial J4: {math.degrees(q0[3]):.2f} deg", flush=True)
        print(f"Target J4 : {math.degrees(target[3]):.2f} deg", flush=True)

        # Synchronize the driver's command baseline to the stable pose measured
        # immediately before enabling, avoiding a stale initialization sample.
        driver.last_command = q0.copy()
        driver.start()
        started = True

        # Enabling can update the motor feedback used internally by the
        # driver. Re-anchor both the trajectory and delta checker to that
        # freshly measured, still-uncommanded pose.
        q0 = driver.fetch_position(refresh=True).copy()
        driver.last_command = q0.copy()
        target = q0.copy()
        target[3] += DELTA
        print("Enabled qpos:", np.array2string(q0, precision=5), flush=True)
        print(f"Enabled/target J4: {math.degrees(q0[3]):.2f} -> "
              f"{math.degrees(target[3]):.2f} deg", flush=True)
        previous = q0.copy()
        peak = np.zeros(8)
        j4_samples = []

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
            j4_samples.append(float(torque[3]))
            time.sleep(0.02)

        for _ in range(50):
            command(q0)
        for alpha in np.linspace(0.0, 1.0, 401)[1:]:
            command(q0 + alpha * (target - q0))

        reached = previous.copy()
        lift_end = len(j4_samples)
        print(
            f"Reached J4 delta: {math.degrees(reached[3] - q0[3]):+.2f} deg",
            flush=True,
        )
        print(f"J4 lift peak torque: {max(abs(x) for x in j4_samples):.3f} Nm", flush=True)

        for _ in range(100):
            command(target)
        hold_values = j4_samples[lift_end:]
        print(
            f"J4 hold mean/peak torque: "
            f"{np.mean(hold_values):+.3f}/{max(abs(x) for x in hold_values):.3f} Nm",
            flush=True,
        )

        for alpha in np.linspace(1.0, 0.0, 401)[1:]:
            command(q0 + alpha * (target - q0))
        for _ in range(50):
            command(q0)

        error = previous - q0
        print("Return error:", np.array2string(error, precision=5), flush=True)
        print("All-joint peak torque:", np.array2string(peak, precision=3), flush=True)
        if np.any(np.abs(error) > 0.025):
            raise RuntimeError(f"return error exceeds 0.025 rad: {error}")
        print("SUCCESS: left J4 lift completed and returned.", flush=True)
        return 0
    finally:
        if started:
            driver.stop()
        else:
            try:
                driver.openarm.disable_all()
            except Exception:
                pass
        print("Safety cleanup: left arm disabled.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
