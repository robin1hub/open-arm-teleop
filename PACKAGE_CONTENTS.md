# Portable package contents

This archive is a source deployment package, not a byte-for-byte disk image.

Included:

- all current project source files and local modifications;
- OpenArm 1.0 MuJoCo models in `models/openarm_v1`;
- all packaged Dora nodes in `nodes`;
- current robot/CAN configuration files;
- simulation, VIVE and physical-control launchers;
- calibration, verification and regression scripts;
- historical controller snapshots in `archives`;
- debugging and operator documentation;
- `requirements-teleop-lock.txt` and `bootstrap_portable.sh`;
- the detailed agent handoff in `AGENT_HANDOFF_DEPLOYMENT.md`.

Intentionally excluded:

- `.venv` — 833 MB and contains absolute paths and host-specific binaries;
- `.git` directories — deployment does not need repository history;
- `__pycache__`, `.pyc`, test caches and generated `out` sessions;
- transient editor, socket and temporary files.

The original workspace was approximately 884 MB. Most of that was the
non-portable virtual environment. Run `bootstrap_portable.sh` on the target
machine to create a fresh environment.

