#!/usr/bin/env python3
"""Return the right arm to the pose recorded before the aborted action."""

import time
import numpy as np
from openarm_driver import Config, SingleArmDriver

TARGET = np.array([-0.00820, 0.53724, -1.61447, 1.79244, -0.01965, -0.36423, 1.44090, -0.01164])
CAPS = np.array([10.0, 10.0, 4.0, 10.0, 2.0, 2.0, 2.0, 2.0])

driver = SingleArmDriver("right_arm", Config("openarm_safe_current.yaml"))
started = False
try:
    q0 = driver.fetch_position(refresh=True)
    if not np.all(np.isfinite(q0)) or np.any(np.abs(TARGET - q0) > 0.15):
        raise RuntimeError(f"unsafe recovery displacement: {TARGET - q0}")
    print("Recovery delta deg:", np.round(np.degrees(TARGET - q0), 2))
    driver.start(); started = True
    previous = q0.copy(); peak = np.zeros(8)
    for a in np.linspace(0.0, 1.0, 251):
        cmd = q0 + a * (TARGET - q0)
        driver.send_position(cmd)
        state = driver.fetch_state(refresh=True)
        q, tau = state["qpos"], state["qtorque"]
        if np.any(~np.isfinite(q)) or np.any(np.abs(q - previous) > 0.05):
            raise RuntimeError("invalid feedback during recovery")
        if np.any(np.abs(cmd - q) > 0.15):
            raise RuntimeError(f"recovery following error: {cmd-q}")
        if np.any(np.abs(tau) > CAPS):
            j = int(np.argmax(np.abs(tau) / CAPS))
            raise RuntimeError(f"J{j+1} torque {tau[j]:.3f} Nm exceeds recovery limit")
        previous = q.copy(); peak = np.maximum(peak, np.abs(tau)); time.sleep(0.02)
    print("Recovery error:", np.round(previous - TARGET, 5))
    print("Peak torque:", np.round(peak, 3))
    print("SUCCESS: pre-action pose restored.")
finally:
    if started: driver.stop()
    else:
        try: driver.openarm.disable_all()
        except Exception: pass
    print("Safety cleanup: right arm disabled.")
