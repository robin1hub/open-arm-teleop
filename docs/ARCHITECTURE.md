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

`vive_absolute_mujoco_teleop.py` implements the supported simulation path.
`vive_absolute_mujoco_real_teleop.py` adds real feedback, output gating, target
resynchronization, and motion guards. The `run_vive_shared_*` launchers provide
the currently validated shared-controller settings.

## Safety boundaries

The system uses several independent software checks:

1. The physical launcher requires `--confirm-hardware`.
2. Both expected CAN interfaces must exist and be `UP`.
3. Physical command output starts disabled.
4. Joint targets are constrained by model limits and a configurable margin.
5. Per-command joint steps, acceleration, and gripper steps are limited.
6. Operators can disable and resynchronize targets without exiting.

These controls reduce common integration risks but do not replace a safety
controller, physical emergency stop, guarded workspace, or a risk assessment.

## Source layout

- Top-level `vive_*` and `interactive_*` modules are the active controllers.
- `safe_kinematics.py` contains shared IK and limit logic.
- `models/openarm_v1` contains the active MuJoCo model assets.
- `nodes/` contains flattened source snapshots of dora/OpenArm components. They
  are intentionally included in the clone and installed by
  `bootstrap_portable.sh`; they are not Git submodules.
- `dataflow*.yaml` contains current and legacy dora dataflow definitions.
- `tests/` contains offline behavioral and safety regressions.
- `archives/` and explicitly labelled legacy files are historical references,
  not supported physical-control entry points.

## Configuration

`openarm_safe_raw_zero.yaml` is the active physical-control configuration used
by the shared real launcher. Controller defaults are visible in
`run_vive_shared_mujoco_real.sh`; changing them affects real motion and should
be reviewed together with the offline safety tests and a low-speed hardware
validation plan.
