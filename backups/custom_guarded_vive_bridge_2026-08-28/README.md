# Custom guarded VIVE bridge backup

Snapshot taken on 2026-08-28 immediately before replacing the local physical
motion filters with the official `openarm-driver==0.3.0` safety pipeline and
velocity profile. Paths below this directory mirror their original locations.

This backup contains the local 0.03-rad command guard, physical soft margins,
collision validation, tracking-error stop, acceleration ramp, slow J8 handling,
launcher settings, tests, configs, and the matching documentation.

Do not execute the backed-up launcher from this directory. Restore files to
their original paths first if a rollback is intentionally required.
