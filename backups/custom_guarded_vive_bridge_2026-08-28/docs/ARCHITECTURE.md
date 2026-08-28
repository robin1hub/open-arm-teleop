# Architecture

## Supported teleoperation path

```text
VIVE controllers
      │ OpenVR poses and buttons
      ▼
VIVE mapping and calibration
      │ desired end-effector poses
      ▼
MuJoCo model + constrained IK
      │ guarded joint targets
      ├──────────────► MuJoCo visualization
      │
      ▼
rate / acceleration / step / joint-limit guards
      │ explicit enable gate
      ▼
OpenArm driver ──► CAN-FD ──► right and left arms
```

`src/openarm_teleop/vive_absolute_mujoco_teleop.py` implements the supported simulation path.
`src/openarm_teleop/vive_absolute_mujoco_real_teleop.py` adds real feedback, output gating, target
resynchronization, and motion guards. The `scripts/launch/run_vive_shared_*`
launchers provide
the current guarded shared-controller settings. The real launcher requests a
phase-locked 60 Hz command cadence while retaining an approximately 0.50 rad/s
J1-J7 ceiling. These settings have offline regression coverage; confirm actual
cadence and motion with a small-amplitude physical validation before normal use.

## Safety boundaries

The system uses several independent software checks:

1. The physical launcher requires `--confirm-hardware`.
2. Both expected CAN interfaces must exist and be `UP`.
3. Physical command output starts disabled.
4. Joint targets are constrained by model limits and a configurable margin.
5. Per-command joint steps, acceleration, and gripper steps are limited.
6. Operators can disable and resynchronize targets without exiting.
7. Each position command uses the feedback already collected by the driver;
   cadence and peak following error are reported once per second.

These controls reduce common integration risks but do not replace a safety
controller, physical emergency stop, guarded workspace, or a risk assessment.

## Source layout

- `src/openarm_teleop/` contains the active VIVE and interactive controllers.
- `src/openarm_teleop/safe_kinematics.py` contains shared IK and limit logic.
- `models/openarm_v1` contains the active MuJoCo model assets.
- `nodes/` contains flattened source snapshots of dora/OpenArm components. They
  are intentionally included in the clone and installed by
  `bootstrap_portable.sh`; they are not Git submodules.
- `dataflows/` contains current and legacy dora dataflow definitions.
- `tests/` contains offline behavioral and safety regressions.
- `archives/` and explicitly labelled legacy files are historical references,
  not supported physical-control entry points.

## Configuration

`config/openarm_safe_raw_zero.yaml` is the active physical-control configuration used
by the shared real launcher. Controller defaults are visible in
`scripts/launch/run_vive_shared_mujoco_real.sh`; changing them affects real
motion and should
be reviewed together with the offline safety tests and a low-speed hardware
validation plan.
