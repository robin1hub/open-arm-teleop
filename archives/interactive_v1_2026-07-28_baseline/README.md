# OpenArm 1.0 interactive-control baseline

Frozen on 2026-07-28 before beginning the second interaction-design iteration.

This snapshot contains:

- OpenArm v1/v2 selectable MuJoCo end-effector drag UI;
- strict limit-preserving Mink IK;
- translation/rotation orientation-cost switching;
- dual-arm selection;
- guarded `E` enable and one-shot `D` physical execution;
- OpenArm 1.0 raw-zero driver configuration;
- v1 physical/MuJoCo mapping and unreachable-target tests;
- the matching operating document.

The OpenArm v1 model assets are not duplicated here. They remain under
`../../models/openarm_v1/` and are identified in the archived program by their
workspace-relative path.

Verify this snapshot:

```bash
sha256sum -c SHA256SUMS
```

To restore, copy the required files back to the repository root only after
preserving any newer work. Do not restore or launch physical control without
re-running the archived tests and checking CAN/robot state.
