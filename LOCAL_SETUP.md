# Local OpenArm 1.0 setup

This workstation is configured for a bimanual OpenArm 1.0 and MuJoCo.
The old Quest/VR dataflows were developed against a v2 model and must not
control the physical robot until they are migrated to v1.

## Interactive MuJoCo control

Simulation only:

```bash
.venv/bin/python interactive_mujoco_ee_drag.py --model-version v1
```

Read-only physical/MuJoCo mapping check:

```bash
.venv/bin/python verify_real_mujoco_alignment_v1.py
```

Physical interactive mode:

```bash
./run_interactive_real.sh --confirm-hardware
```

See `INTERACTIVE_CONTROL.md` before enabling either arm.

## Simulation-only action test

To verify the joint-action path without VR, CAN, or a physical robot:

```bash
./run_action_test_mujoco.sh
```

The bundled action source publishes one `float32[8]` target per arm (seven
arm joints followed by the gripper). Both simulated arms move from `home` to
mirrored targets over three seconds and then return continuously.

## Legacy MuJoCo and Quest 3

This flow has not yet been migrated to the OpenArm 1.0 model. Simulation use
is allowed for development, but do not route its output to the physical arms.

1. Put the PC and Quest 3 on the same LAN.
2. In the OpenArm Quest app, set the host shown by `./run_mujoco_vr.sh`
   and UDP port `5006`.
3. Run `./run_mujoco_vr.sh`.
4. Open <http://127.0.0.1:8000> for recording status.

Recorded simulation episodes are written below `vr_mujoco_data/`.

## Physical robot

The verified OpenArm 1.0 driver mapping is:

- right arm: `can0`
- left arm: `can1`

After connecting both CAN-FD adapters, configure and inspect them without
enabling motors:

```bash
sudo openarm-can-cli can_configure
ip -details link show can0
ip -details link show can1
openarm-can-cli -i can0 discover
openarm-can-cli -i can1 discover
```

Each bus should discover the expected eight motor IDs before proceeding.
`run_real_vr.sh` and `dataflow-vr-real-no-camera.yaml` still contain the old
v2 model assumptions. Do not use them for physical control until their v1
migration is complete.
