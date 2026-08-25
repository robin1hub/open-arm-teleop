# Quick start

This guide gets a new Ubuntu 24.04 workstation from a clean clone to MuJoCo
simulation, then describes the additional gates for VIVE and physical hardware.

## 1. Install prerequisites

Use an Ubuntu on Xorg login session for SteamVR. Install the project tools and
runtime libraries:

```bash
sudo apt update
sudo apt install python3.12 python3.12-venv python3-pip git \
  can-utils iproute2 usbutils libgl1 libglfw3 libusb-1.0-0
```

Steam and SteamVR are only required for VIVE operation. Install them through
the normal Steam client and verify that the headset and both controllers are
green in the SteamVR status window before starting this project.

## 2. Create the Python environment

```bash
git clone https://github.com/robin1hub/open-arm-teleop.git
cd open-arm-teleop
./bootstrap_portable.sh
```

The script creates `.venv`, installs the pinned teleoperation dependencies,
installs the local nodes in editable mode, runs the primary safety regression,
and compiles the controller entry points. Re-run it after dependency changes.

## 3. Validate without hardware

Run all offline tests:

```bash
.venv/bin/python -m unittest -q \
  tests/test_vive_real_soft_limits.py \
  tests/test_safe_ik_limits.py \
  tests/test_ee_to_joint_mujoco.py
```

Then verify the simulation action path:

```bash
./run_action_test_mujoco.sh
```

The two simulated arms should move smoothly from home to mirrored targets and
back. Stop here and resolve errors before adding VR or physical hardware.

## 4. Run VIVE simulation

Start SteamVR first, keep both controllers awake and visible to the base
stations, then run:

```bash
./run_vive_shared_mujoco_sim.sh
```

Press `K` to calibrate and use the controller trigger (or `C`) to capture a
reference pose. Begin with small, slow motions. `Esc` exits.

If the launcher reports that `vrserver` is not running, start SteamVR in the
same desktop session. See [Troubleshooting](TROUBLESHOOTING.md) for display and
tracking checks.

## 5. Add the physical arms

Read [Hardware safety](HARDWARE_SAFETY.md) in full. With motor output still
disabled, configure and inspect both CAN buses:

```bash
sudo openarm-can-cli can_configure
ip -details link show can0
ip -details link show can1
openarm-can-cli -i can0 discover
openarm-can-cli -i can1 discover
```

Confirm that each bus is `UP`, uses CAN-FD at the expected rates, is
`ERROR-ACTIVE`, and discovers the correct eight motor IDs. The normal local
mapping is right=`can0`, left=`can1`, but USB enumeration can change after a
replug. Identify buses from returned motor identities, not interface names
alone.

Start the guarded physical controller:

```bash
./run_vive_shared_mujoco_real.sh --confirm-hardware
```

Physical output remains disabled at startup. Calibrate (`K`), capture a
reference (`C` or trigger), compare the real and simulated pose, then use `E`
only when the workspace is clear and the emergency stop is reachable.

## 6. Stop safely

Press `E` to disable output before removing the headset or putting down a
controller. Press `Esc` to cleanly close the program. If tracking, CAN feedback,
or motion becomes abnormal, disable output or use the physical emergency stop;
do not attempt to recover by increasing limits.
