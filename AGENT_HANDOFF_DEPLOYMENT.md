# OpenArm 1.0 + VIVE Pro 2 deployment and agent handoff

## 1. Purpose and current state

This package is the current working copy from `/home/robin/open_arm`, prepared
on 2026-08-11. It combines the upstream Dora OpenArm data-collection project
with a locally developed single-process MuJoCo/VIVE teleoperation stack.

The physical robot was initially mistaken for OpenArm 2.0. It has since been
confirmed to be **OpenArm 1.0**. Treat old mentions of 2.0 in early debugging
notes as historical. The current V1 model and current launcher/config files are
authoritative.

The current main entry points are:

```bash
./run_vive_shared_mujoco_sim.sh
./run_vive_shared_mujoco_real.sh --confirm-hardware
```

The first is simulation-only. The second can command the physical robot and
starts disabled.

## 2. Known-good source machine

```text
OS:             Ubuntu 24.04.4 LTS
Kernel:         7.0.0-28-generic
Session:        GNOME on Xorg (X11)
Python:         3.12.3
MuJoCo:         3.10.0
SteamVR:        Linux installation through Steam
Headset:        HTC VIVE Pro 2
Controllers:    two VIVE controllers
Base stations:  SteamVR 2.0 tracking
Robot:          OpenArm 1.0, dual arm, no torso mechanism
CAN adapters:   two DM-USB2FDCAN/gs_usb-compatible adapters
```

SteamVR tracking was unreliable or unavailable in the previous GNOME Wayland
session, so use **Ubuntu on Xorg**. A connected VIVE DisplayPort can also cause
Ubuntu Dock flashing after SteamVR exits. This is a GNOME/AMD display hotplug
issue, not an OpenArm process; see the operational notes below.

## 3. Physical robot identity and mapping

```text
right arm: can0
left arm:  can1
motor IDs per arm:       1 through 8
motor feedback IDs:      0x11 through 0x18
mode:                    MIT
CAN arbitration bitrate: 1 Mbps
CAN-FD data bitrate:     5 Mbps
```

Both arms were mechanically placed in the OpenArm 1.0 documented zero posture
and the motor zero was written there. The active configuration therefore uses
zero software offsets:

```text
openarm_safe_raw_zero.yaml
```

Do not reuse that calibration blindly on a different physical robot. Verify
every joint direction and zero position with motors disabled first.

The adapter driver does not support `restart-ms`. If the official CAN setup
command fails with `Device doesn't support restart from Bus Off`, configure the
interfaces without `restart-ms`. The known-good link parameters are CAN-FD,
1M/5M, arbitration sample point 0.75 and data sample point 0.75. Read
`CAN_AND_ARM_DEBUGGING.md` before changing motor IDs or CAN parameters.

## 4. Architecture

The current primary controller is not a complete Dora graph. It is a direct
Python pipeline:

```text
SteamVR/OpenVR
  -> HMD and controller poses
  -> three-stage body calibration
  -> shared absolute controller-tip poses
  -> raw left/right end-effector targets
  -> safe IK and collision projection
  -> J1-J7 plus J8 gripper targets
  -> MuJoCo visualization
  -> optional guarded physical bridge
  -> openarm_driver
  -> can0/can1
  -> DM motors 1-8
```

Important files:

- `interactive_mujoco_ee_drag.py`: V1 model loading, joint resolver, FK helpers,
  physical/model conversion, collision checks and legacy mouse control.
- `safe_kinematics.py`: strict Mink differential IK that always retains
  configuration limits and anchors redundant solutions to the current posture.
- `vive_mujoco_teleop.py`: retained relative/clutched VIVE controller.
- `vive_absolute_mujoco_teleop.py`: absolute/shared calibration, OpenVR input,
  controller-tip mapping, bimanual target projection, gripper input and MuJoCo
  loop.
- `vive_mujoco_real_teleop.py`: `BimanualHardwareBridge`, feedback reads,
  enable/disable, joint velocity/acceleration limiting and tracking checks.
- `vive_absolute_mujoco_real_teleop.py`: physical subclass, E/P/H keys,
  physical soft limits, tracking-loss handling and command-shadow rendering.
- `openarm_safe_raw_zero.yaml`: active physical limits, CAN mapping, IDs and
  gains.
- `models/openarm_v1`: packaged V1 MuJoCo model used by the local controller.

The upstream modular Dora nodes remain under `nodes/` and the YAML dataflows
remain available, but they are not the current recommended VIVE control path.

## 5. Target-machine installation

### 5.1 Install system packages

Install at least:

```bash
sudo apt update
sudo apt install -y \
  python3.12 python3.12-venv python3-pip git \
  can-utils iproute2 usbutils \
  libgl1 libglfw3 libusb-1.0-0
```

Install Steam and SteamVR separately through the normal Ubuntu/Steam workflow.
Log into an **Xorg** desktop session before configuring VIVE tracking.

### 5.2 Extract and build the Python environment

```bash
tar -xzf open_arm_portable_2026-08-11.tar.gz
cd open_arm_portable_2026-08-11
chmod +x bootstrap_portable.sh run_*.sh
./bootstrap_portable.sh
```

`bootstrap_portable.sh` creates a new `.venv`, installs the locked teleoperation
dependencies, installs the packaged local nodes in editable mode, compiles the
main scripts and runs the offline physical safety tests.

The exact `openarm_control` Git commit used here is
`e25c0f3c7e0d60fbdb1000c25e689b0c014e2f54`.

### 5.3 Validate simulation without SteamVR

First validate imports and IK:

```bash
.venv/bin/python -m unittest -v tests/test_vive_real_soft_limits.py
.venv/bin/python tests/test_ee_to_joint_mujoco.py
```

Then install/start SteamVR and verify the headset and both controllers are
green/tracked. Pure shared-space simulation is:

```bash
./run_vive_shared_mujoco_sim.sh
```

Press `K`, then capture these poses with a controller trigger, releasing it
between poses:

1. face forward;
2. both arms naturally down with neutral wrists;
3. horizontal T-pose.

After calibration, controller side grips act as per-arm motion permissions.
The touchpad controls the grippers. Pure simulation has no `E` hardware enable.

### 5.4 Validate CAN without moving the robot

Do not connect to a robot with unknown zero positions and immediately run the
physical controller. First inspect both interfaces and motor feedback using
the procedure in `CAN_AND_ARM_DEBUGGING.md`.

Expected interface details:

```text
FD enabled
bitrate 1000000
dbitrate 5000000
state ERROR-ACTIVE
```

Verify right/left adapter enumeration on the new computer. Linux can assign
`can0` and `can1` differently after replugging. The YAML mapping must match the
physical cables, not merely the interface names from this source computer.

Run the read-only V1 alignment check with motors disabled:

```bash
.venv/bin/python verify_real_mujoco_alignment_v1.py
```

Confirm every J1-J7 physical posture matches MuJoCo. Confirm J8 direction and
range separately because V1 hardware reports gripper motor angle while the
MuJoCo model uses finger slide distance.

### 5.5 Physical shared-space control

Only after simulation, CAN and alignment checks pass:

```bash
./run_vive_shared_mujoco_real.sh --confirm-hardware
```

Current launcher parameters:

```text
Cartesian target step:       9 mm/frame
orientation target step:     3.5 deg/frame
CAN command request:         40 Hz
J1-J7 maximum command step:  0.0125 rad/cycle (~0.50 rad/s)
J1-J7 acceleration limit:    3.0 rad/s^2
J8 maximum command step:     0.001 rad/cycle
physical soft-limit margin:  0.04 rad
tracking-error threshold:    0.15 rad
tracking-loss timeout:       0.30 s
```

Keys:

- `K`: start/restart three-step calibration; physical motors are disabled.
- trigger or `C`: capture the current calibration pose.
- `E`: enable or disable both arms. Release both side grips before enabling.
- `P`: disable, read physical posture into MuJoCo and clear calibration.
- `H`: rate-limited physical return toward the configured home posture; it is
  not an emergency stop.
- `Esc`: close and disable during cleanup.

Keep the physical emergency stop reachable. First enable without holding either
side grip, then test one arm by only a few millimetres.

## 6. Safety policy and known failure classes

The real mode deliberately has more failure paths than simulation:

- invalid or stale CAN feedback;
- hard joint-limit violation;
- physical soft-limit rejection;
- controller tracking lost while its grip is held for at least 0.30 s;
- motor command/feedback error above 0.15 rad;
- predicted self-collision, inter-arm collision or arm/torso collision.

Known incidents observed on the source robot include:

- right hand predicted to collide with `openarm_body_link0_collision`;
- both grippers predicted to collide;
- left J8/gripper tracking error because the gripper cannot follow arm-joint
  speed; this is why J8 has its own slow limit;
- SteamVR one-frame and sustained `NO-POSE` events;
- numerical mismatch at a truncated YAML joint boundary; this is why a 0.04
  rad physical soft margin is retained.

Simulation usually projects or freezes an invalid target and continues. Real
mode currently disarms both arms for several physical safety failures. Do not
increase thresholds merely to make it feel as smooth as simulation.

## 7. Recent controller changes

The current real controller renders the rate-limited joint posture actually
sent to hardware instead of instantly rendering the final IK result. The next
IK starting point is synchronized to this command posture to prevent the
discontinuity guard from permanently rejecting later frames.

Safe target projection was also changed so a position-only fallback cannot
starve wrist orientation. If position projection succeeds without orientation,
the same frame attempts one additional orientation-only, collision-checked
step.

These changes compile and `tests/test_vive_real_soft_limits.py` currently has six
passing tests covering soft limits, gripper limiting, acceleration ramping,
endpoint non-overshoot and reversal braking.

## 8. Important unfinished work

### 8.1 Human-like elbow behavior

The robot has seven arm joints, so one end-effector pose has multiple redundant
solutions. Current IK constrains end-effector position/orientation and weakly
anchors the solution to the previous posture. It does **not** constrain elbow
position or elbow-plane direction. Therefore the elbow may be mathematically
valid but unlike the human operator.

Do not attempt to fix this only by reducing J5-J7 participation: that can move
the unwanted rotation into the shoulder. The intended next design is a low- or
medium-weight elbow position/direction task derived from calibrated shoulder,
controller and estimated human arm geometry, while retaining end-effector,
joint-limit and collision priorities.

### 8.2 Real-time execution

The physical output currently runs from the MuJoCo/UI loop and requests 40 Hz;
feedback and rendering work can reduce effective cadence. A future improvement
is a dedicated fixed-rate hardware command thread with a timestamped target
buffer, while all enable/disable and safety state transitions remain atomic.

### 8.3 Recoverable versus fatal constraints

IK projection and some unreachable targets are recoverable, while hard limits,
CAN loss and sustained tracking errors should remain fatal. The current real
policy still disarms both arms for some predicted-collision cases that could
instead freeze the affected side and continue safe projection. Any change must
be tested in simulation and then at low speed with the emergency stop ready.

## 9. VIVE/GNOME operational note

After SteamVR exits, the VIVE Pro 2 DisplayPort may be returned to Xorg as a
rapidly changing/ghost display. On the source AMD laptop this triggered Ubuntu
Dock errors such as:

```text
Spurious clutter_actor_allocate ... dashtodockContainer
```

and caused the Dock to flash. Disconnecting the headset DisplayPort, reloading
GNOME Shell on X11, or disabling/re-enabling Ubuntu Dock resolves the symptom.
It is not caused by OpenArm or a remaining teleoperation process.

## 10. Documentation map

- `YANGHAO_OPENARM_CONTROL.md`: concise current launcher guide.
- `INTERACTIVE_CONTROL.md`: detailed evolution and operating instructions.
- `CAN_AND_ARM_DEBUGGING.md`: field-tested CAN-FD, IDs and motor diagnostics.
- `README_YH.md`: local documentation index.
- `WINDOWS_DAMIAO_MOTOR_ID_SETUP.md`: Windows/UART motor-ID procedure.
- `WINDOWS_VIVE_BRIDGE_GUIDE.md`: historical VIVE bridging notes.
- `PACKAGE_CONTENTS.md`: included/excluded archive contents.
- `archives/`: preserved older interactive controller versions.

## 11. Rules for the next agent

1. Read this file, `INTERACTIVE_CONTROL.md` and
   `CAN_AND_ARM_DEBUGGING.md` before changing physical control.
2. Treat the current uncommitted/local files as user work; do not reset or
   replace them with upstream copies.
3. Confirm OpenArm **1.0** model use in every new entry point.
4. Keep simulation and physical launchers separate and require explicit
   `--confirm-hardware` for real motion.
5. Never infer `can0/can1` identity after moving to a new computer; verify the
   cable/interface mapping.
6. Read physical posture before enabling and synchronize MuJoCo to it.
7. Preserve hard joint limits, collision checking and emergency-stop access.
8. Test offline, then simulation, then disabled/read-only hardware, then one
   arm at low speed before bimanual movement.
9. Record the exact `SAFETY STOP` line before modifying any threshold.
10. Do not copy the old `.venv`; recreate it with `bootstrap_portable.sh`.
