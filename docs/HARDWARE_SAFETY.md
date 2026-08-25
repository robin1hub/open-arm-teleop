# Hardware safety checklist

This project controls high-torque moving hardware. It is experimental software,
not a certified safety system. The operator is responsible for the robot,
workspace, and compliance with applicable rules.

## Before every physical session

- Mount both arms securely and remove loose tools, cables, and obstacles.
- Keep all people outside the reachable workspace.
- Make the physical emergency stop immediately reachable and test it according
  to the hardware manufacturer's procedure.
- Inspect links, fasteners, grippers, cabling, CAN adapters, and power wiring.
- Move each unpowered joint only when the hardware procedure permits it; do not
  force a resistant axis.
- Start SteamVR and confirm stable tracking for both controllers.
- Run the task in simulation and verify direction, scale, limits, and controls.
- Configure CAN-FD, confirm both links are `UP` and `ERROR-ACTIVE`, and discover
  the expected motor IDs on each physical arm.
- Re-identify right and left buses after any USB disconnect or reboot.
- Confirm the real launcher starts with output disabled.

## First motion after a change

- Use a clear workspace and the lowest practical speed and travel.
- Test one small, isolated motion before combined or full-range motion.
- Watch feedback and the real robot, not only the simulation window.
- Keep one hand ready to disable output or press the emergency stop.
- Stop on unexpected sound, heat, vibration, tracking error, delayed feedback,
  bus error, limit contact, or rising mechanical resistance.

Never raise torque, current, speed, acceleration, tracking-error, or joint-step
limits to push through an unexplained failure. Diagnose the cause with output
disabled. A transient CAN query may be retried once, but repeated failures must
be treated as a fault.

## During teleoperation

- Avoid controller handoffs while physical output is enabled.
- Disable output before removing the headset, setting down a controller, or
  entering the workspace.
- Use `P` to disable and resynchronize after tracking loss or a pose jump.
- Do not rely on the software window as an emergency stop.

## After the session

- Disable output, exit the controller cleanly, and remove robot power according
  to the hardware procedure.
- Record unexpected behavior and preserve logs before attempting another run.
- Inspect the robot after any collision, overload, emergency stop, or bus fault.

## Pull requests affecting physical motion

Changes to joint mapping, signs, limits, kinematics, command frequency, velocity,
acceleration, gripper range, homing, CAN configuration, or output gating must
include offline tests and a written hardware validation plan. Do not claim a
change is hardware-tested unless a responsible operator performed and recorded
that test.
