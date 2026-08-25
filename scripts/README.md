# Scripts

- `launch/` contains supported and legacy application entry points. Launchers
  resolve the repository root automatically and may be run from any directory.
- `diagnostics/` contains read-only alignment checks. Confirm a script's header
  before connecting hardware.
- `hardware_tools/` contains narrowly scoped maintenance and recovery tools.
  Some enable or move motors and must only be used by an authorized operator
  after reviewing the source and hardware safety checklist.

One-off local experiments belong in the ignored `hardware_diagnostics/`
directory and should not be committed.
