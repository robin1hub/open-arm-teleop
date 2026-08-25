# OpenArm 1.0 VIVE teleoperation and data collection

This repository is a portable OpenArm 1.0 development snapshot containing
MuJoCo simulation, VIVE/SteamVR teleoperation, guarded dual-arm physical
control, CAN-FD diagnostics, and the original dora-rs data-collection nodes.

Physical control is disabled by default and requires explicit hardware
confirmation. Keep the emergency stop accessible, verify both CAN interfaces,
and validate motion in simulation before enabling the robot.

## OpenArm 1.0 control

Search `YANGHAO` to locate the current local simulation and physical-arm
workflow. Start with
[`YANGHAO_OPENARM_CONTROL.md`](YANGHAO_OPENARM_CONTROL.md).

This repository provides data collection configurations for [OpenArm](https://openarm.dev/) with [dora-rs](https://dora-rs.ai/).

## Configurations

[`metadata.yaml`](metadata.yaml) is metadata used by all configurations.

### Real configuration

TODO

### Dummy configuration

[`dataflow_dummy.yaml`](dataflow_dummy.yaml) is a configuration that doesn't use real OpenArm. We can use this for testing a dataflow without real OpenArm.

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
