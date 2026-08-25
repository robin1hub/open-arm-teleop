#!/usr/bin/env python3
"""Synchronized bimanual J4 98->138 degree lift with torque logging."""

import math
import time

import numpy as np
from openarm_driver import Config, SingleArmDriver


PREP_J4 = math.radians(98.0)
LIFT_J4 = math.radians(138.0)
MAX_TORQUES = np.array([10.5, 10.5, 4.0, 12.0, 2.0, 2.0, 2.0, 2.0])


def main() -> int:
    cfg = Config("openarm_safe_current.yaml")
    drivers = {
        "right": SingleArmDriver("right_arm", cfg),
        "left": SingleArmDriver("left_arm", cfg),
    }
    started = []
    try:
        initial = {}
        for side, driver in drivers.items():
            samples = []
            for _ in range(10):
                samples.append(driver.fetch_position(refresh=True))
                time.sleep(0.02)
            samples = np.asarray(samples)
            if not np.all(np.isfinite(samples)):
                raise RuntimeError(f"{side}: non-finite initial feedback")
            if np.max(np.ptp(samples[-5:], axis=0)) > 0.01:
                raise RuntimeError(f"{side}: unstable initial feedback")
            initial[side] = samples[-1].copy()
            driver.last_command = initial[side].copy()
            print(
                f"{side} initial J4: {math.degrees(initial[side][3]):.2f} deg",
                flush=True,
            )

        # Enable both without commanding a move, then re-anchor to fresh state.
        for side in ("right", "left"):
            drivers[side].start()
            started.append(side)

        origins = {}
        previous = {}
        peaks = {side: np.zeros(8) for side in drivers}
        j4_log = {side: [] for side in drivers}
        for side, driver in drivers.items():
            origins[side] = driver.fetch_position(refresh=True).copy()
            driver.last_command = origins[side].copy()
            previous[side] = origins[side].copy()
            print(
                f"{side} enabled J4: {math.degrees(origins[side][3]):.2f} deg",
                flush=True,
            )

        prep = {side: q.copy() for side, q in origins.items()}
        lift = {side: q.copy() for side, q in origins.items()}
        for side in drivers:
            prep[side][3] = PREP_J4
            lift[side][3] = LIFT_J4

        def command(targets: dict[str, np.ndarray], phase: str) -> None:
            for side in ("right", "left"):
                driver = drivers[side]
                qcmd = targets[side]
                driver.send_position(qcmd)
                state = driver.fetch_state(refresh=True)
                q, torque = state["qpos"], state["qtorque"]
                if not np.all(np.isfinite(q)) or not np.all(np.isfinite(torque)):
                    raise RuntimeError(f"{side} {phase}: non-finite motor state")
                if np.any(np.abs(q - previous[side]) > 0.05):
                    raise RuntimeError(f"{side} {phase}: feedback jump {q-previous[side]}")
                if np.any(np.abs(qcmd - q) > 0.18):
                    raise RuntimeError(f"{side} {phase}: following error {qcmd-q}")
                if np.any(np.abs(torque) > MAX_TORQUES):
                    j = int(np.argmax(np.abs(torque) / MAX_TORQUES))
                    raise RuntimeError(
                        f"{side} {phase}: J{j+1} torque {torque[j]:.3f} Nm "
                        f"exceeds {MAX_TORQUES[j]:.1f} Nm"
                    )
                previous[side] = q.copy()
                peaks[side] = np.maximum(peaks[side], np.abs(torque))
                j4_log[side].append(float(torque[3]))
            time.sleep(0.02)

        # One second takeover hold.
        for _ in range(50):
            command(origins, "takeover")

        # Six-second move to common 98-degree starting angle.
        for alpha in np.linspace(0.0, 1.0, 301)[1:]:
            targets = {
                side: origins[side] + alpha * (prep[side] - origins[side])
                for side in drivers
            }
            command(targets, "prepare")
        for side in drivers:
            print(
                f"{side} prepared J4: {math.degrees(previous[side][3]):.2f} deg",
                flush=True,
            )

        # Eight-second synchronized 40-degree lift.
        lift_start = {side: len(j4_log[side]) for side in drivers}
        for alpha in np.linspace(0.0, 1.0, 401)[1:]:
            targets = {
                side: prep[side] + alpha * (lift[side] - prep[side])
                for side in drivers
            }
            command(targets, "lift")
        hold_start = {side: len(j4_log[side]) for side in drivers}
        for side in drivers:
            segment = j4_log[side][lift_start[side]:]
            print(
                f"{side} reached J4: {math.degrees(previous[side][3]):.2f} deg; "
                f"lift peak {max(abs(x) for x in segment):.3f} Nm",
                flush=True,
            )

        # Two-second raised hold.
        for _ in range(100):
            command(lift, "raised hold")
        for side in drivers:
            segment = j4_log[side][hold_start[side]:]
            print(
                f"{side} J4 hold mean/peak: "
                f"{np.mean(segment):+.3f}/{max(abs(x) for x in segment):.3f} Nm",
                flush=True,
            )

        # Eight-second return to 98 degrees.
        for alpha in np.linspace(1.0, 0.0, 401)[1:]:
            targets = {
                side: prep[side] + alpha * (lift[side] - prep[side])
                for side in drivers
            }
            command(targets, "lower")

        # Six-second restore to each arm's starting pose.
        for alpha in np.linspace(0.0, 1.0, 301)[1:]:
            targets = {
                side: prep[side] + alpha * (origins[side] - prep[side])
                for side in drivers
            }
            command(targets, "restore")
        for _ in range(50):
            command(origins, "final hold")

        for side in drivers:
            error = previous[side] - origins[side]
            print(
                f"{side} return error: {np.array2string(error, precision=5)}",
                flush=True,
            )
            print(
                f"{side} all-joint peak torque: "
                f"{np.array2string(peaks[side], precision=3)}",
                flush=True,
            )
            if np.any(np.abs(error) > 0.025):
                raise RuntimeError(f"{side}: return error exceeds 0.025 rad")
        print("SUCCESS: bimanual J4 40-degree lift completed.", flush=True)
        return 0
    finally:
        for side in reversed(started):
            try:
                drivers[side].stop()
            except Exception as exc:
                print(f"WARNING: {side} stop failed: {exc}", flush=True)
                try:
                    drivers[side].openarm.disable_all()
                except Exception:
                    pass
        for side in drivers:
            if side not in started:
                try:
                    drivers[side].openarm.disable_all()
                except Exception:
                    pass
        print("Safety cleanup: both arms disabled.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
