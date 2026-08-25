# Contributing

Thanks for helping improve OpenArm 1.0 VIVE Teleoperation. Small, focused pull
requests with a reproducible test are easiest to review.

## Before opening a change

1. Search existing issues and pull requests.
2. For a bug, include the operating system, Python version, launch command,
   simulation or hardware mode, expected behavior, actual behavior, and a
   minimal log with credentials and personal data removed.
3. For a new physical-control behavior, open an issue before implementation so
   its safety boundary and validation plan can be discussed.

Do not include passwords, API tokens, SSH keys, private network details,
personal calibration data, generated datasets, or captured images without the
subjects' permission.

## Development setup

```bash
git clone https://github.com/robin1hub/open-arm-teleop.git
cd open-arm-teleop
./bootstrap_portable.sh
```

Run the offline regression suite:

```bash
.venv/bin/python -m unittest -q \
  tests/test_vive_real_soft_limits.py \
  tests/test_safe_ik_limits.py \
  tests/test_ee_to_joint_mujoco.py
```

Also run the smallest relevant simulation scenario. Hardware access is not
required for most contributions and contributors must not claim hardware
validation that they did not perform.

## Pull request expectations

- Explain the problem, approach, user-visible effect, and remaining risks.
- Add or update documentation for changed commands, controls, or configuration.
- Add offline coverage for controller, IK, mapping, or safety-limit changes.
- Keep generated files, virtual environments, recordings, and local diagnostics
  out of commits.
- Preserve upstream copyright and license notices.

Changes affecting physical motion must follow the additional requirements in
[Hardware safety](docs/HARDWARE_SAFETY.md#pull-requests-affecting-physical-motion).

By participating, you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).
Unless stated otherwise, contributions are accepted under the repository's
Apache License 2.0.
