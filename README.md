# OpenArm 1.0 VIVE Teleoperation

An experimental dual-arm teleoperation stack for OpenArm 1.0. It combines
SteamVR/VIVE tracking, MuJoCo visualization and inverse kinematics, guarded
CAN-FD control of a physical robot, and dora-rs data-collection nodes.

> [!WARNING]
> This is an independent development snapshot, not an official OpenArm release.
> Physical robot operation can cause injury or damage. Start in simulation,
> keep the emergency stop within reach, clear the workspace, and follow the
> [hardware safety checklist](docs/HARDWARE_SAFETY.md). The authors and
> contributors provide no safety certification.

中文用户可直接阅读[中文快速开始](docs/QUICKSTART_zh-CN.md)。

## What is included

- Absolute and incremental VIVE controller mappings
- Mirrored MuJoCo visualization for both arms
- Rate-, acceleration-, step-, and joint-limit guards for real hardware
- Explicit physical-output enable and CAN-interface preflight checks
- OpenArm 1.0 models and locally packaged dora nodes
- Offline regression tests for IK and motion limits
- Legacy dora-rs recording flows for continued migration work

## Project status

| Path | Status | Hardware required |
| --- | --- | --- |
| MuJoCo action test | Supported | No |
| Interactive MuJoCo control | Supported | No |
| VIVE + MuJoCo teleoperation | Supported | VIVE/SteamVR |
| VIVE + physical OpenArm 1.0 | Experimental, guarded | VIVE, dual OpenArm, two CAN-FD adapters |
| Legacy Quest/dora physical flow | Not migrated to OpenArm 1.0 | Do not connect to a robot |

The currently validated workstation profile is Ubuntu 24.04, Python 3.12,
SteamVR under an Xorg session, and two CAN-FD interfaces. Other Linux systems
may work but have not been validated by this project.

## Quick start: simulation

Install the system packages:

```bash
sudo apt update
sudo apt install python3.12 python3.12-venv python3-pip git \
  can-utils iproute2 usbutils libgl1 libglfw3 libusb-1.0-0
```

Clone the repository and build its isolated Python environment:

```bash
git clone https://github.com/robin1hub/open-arm-teleop.git
cd open-arm-teleop
./bootstrap_portable.sh
```

Run a simulation-only smoke test that requires neither VR nor CAN:

```bash
./run_action_test_mujoco.sh
```

For VIVE simulation, install Steam and SteamVR, pair the headset/controllers,
start SteamVR, and run:

```bash
./run_vive_shared_mujoco_sim.sh
```

See the [complete quick-start guide](docs/QUICKSTART.md) for calibration,
controls, expected results, and troubleshooting.

## Physical robot workflow

Do not begin here on a new installation. First complete the simulation path
and read the [hardware safety checklist](docs/HARDWARE_SAFETY.md).

The default mapping is `can0` for the right arm and `can1` for the left arm.
USB reconnection can swap these names, so verify motor identities every time:

```bash
sudo openarm-can-cli can_configure
ip -details link show can0
ip -details link show can1
openarm-can-cli -i can0 discover
openarm-can-cli -i can1 discover
```

Only after the checks pass and SteamVR is healthy:

```bash
./run_vive_shared_mujoco_real.sh --confirm-hardware
```

The launcher starts with motor output disabled. Calibrate first, then enable
output explicitly from the application. Never use a missing diagnostic response
as a reason to bypass a guard.

## Controls

| Input | Action |
| --- | --- |
| `K` | Capture/calibrate the VIVE-to-robot mapping |
| Controller trigger or `C` | Capture the current reference pose |
| `E` | Enable or disable physical command output |
| `P` | Disable output and resynchronize targets |
| `H` | Request the guarded, rate-limited home motion |
| `Esc` | Stop and clean up |

## Repository map

```text
.
├── docs/                     Public setup, architecture, and safety guides
├── models/openarm_v1/        OpenArm 1.0 MuJoCo assets
├── nodes/                    Vendored dora/OpenArm node sources
├── scripts/                  Maintainer and diagnostic utilities
├── tests/                    Offline regression tests
├── bootstrap_portable.sh     Reproducible Python environment setup
├── run_*                     Supported and legacy launch entry points
└── vive_*_teleop.py          VIVE simulation and physical controllers
```

The node directories are source snapshots rather than Git submodules so a
normal clone is self-contained. Their upstream origins and licenses are listed
in [NOTICE](NOTICE).

## Documentation

- [Quick start](docs/QUICKSTART.md)
- [中文快速开始](docs/QUICKSTART_zh-CN.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Hardware safety](docs/HARDWARE_SAFETY.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Interactive control reference](INTERACTIVE_CONTROL.md)
- [CAN and arm debugging reference](CAN_AND_ARM_DEBUGGING.md)
- [Contributing](CONTRIBUTING.md)
- [Security policy](SECURITY.md)

Files such as `AGENT_HANDOFF_DEPLOYMENT.md` and
`YANGHAO_OPENARM_CONTROL.md` preserve deployment history. They may contain
machine-specific or older procedures; use the documents above as the public
entry points.

## Development checks

After bootstrapping:

```bash
.venv/bin/python -m unittest -q \
  tests/test_vive_real_soft_limits.py \
  tests/test_safe_ik_limits.py \
  tests/test_ee_to_joint_mujoco.py
```

These tests are offline. Passing them does not certify a physical robot for
operation.

## License and attribution

The repository is licensed under the Apache License 2.0 unless a component
states otherwise. See [LICENSE](LICENSE) and [NOTICE](NOTICE). Original OpenArm
and dora node work is copyright its respective upstream contributors. OpenArm,
VIVE, SteamVR, and MuJoCo may be trademarks of their respective owners.

Participation is governed by the [Code of Conduct](CODE_OF_CONDUCT.md).
