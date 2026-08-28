# Troubleshooting

## SteamVR is not detected

The VIVE launchers require the `vrserver` process:

```bash
pgrep -a vrserver
```

Start SteamVR in the same desktop session. On Ubuntu, use an Xorg session if
the compositor or headset view fails. Confirm the headset and controllers are
green in SteamVR before retrying.

## A controller pose freezes or jumps

Restore line of sight to the base stations, wake the controller, and verify its
SteamVR status. In physical mode, disable output and resynchronize with `P`
before capturing a new reference. Do not continue through a tracking jump.

## `can0` or `can1` is missing/down

Inspect USB and network state without enabling motors:

```bash
lsusb
ip -details link show can0
ip -details link show can1
```

Reconnect the adapter if necessary, then run:

```bash
sudo openarm-can-cli can_configure
```

USB enumeration can swap interface names. Run discovery on each interface and
verify the actual arm identity before physical control.

## CAN discovery is incomplete

Retry one transient query. If the result remains incomplete, stop. Check power,
termination, wiring, adapter assignment, CAN-FD bitrate, and error counters with
output disabled. See [CAN and arm debugging](legacy/CAN_AND_ARM_DEBUGGING.md).

## Motion is slow, jerky, or delayed

Check SteamVR tracking stability and CAN feedback first. Compare the same
trajectory in simulation. During physical movement, inspect the once-per-second
line that starts with:

```text
[hardware] cadence target=60.0Hz left=...Hz/peak_err=... right=...Hz/peak_err=...
```

An actively commanded arm should remain close to 60 Hz. A side that is not being
commanded can report 0 Hz. If an active arm stays below 50 Hz, record the line
and stop before changing limits: the UI/IK/collision workload or CAN feedback is
still missing deadlines. If cadence is healthy but peak error rises, inspect the
mechanics, power, wiring, gains, and the affected joint instead of increasing
the tracking-error threshold.

Review command frequency, maximum joint step, and acceleration together;
changing only one can create a new bottleneck. Do not increase limits during an
unexplained mechanical or communication fault.

## Python environment errors

Confirm Python 3.12 is installed, remove no files from the repository, and rerun:

```bash
./bootstrap_portable.sh
```

Use `.venv/bin/python` for direct commands so system packages do not leak into
the project environment.
