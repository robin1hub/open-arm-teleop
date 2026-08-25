# Security policy

## Supported versions

This project is pre-release research software. Security and safety fixes are
applied to the latest `main` branch; older snapshots are not supported.

## Reporting a vulnerability

Do not open a public issue for a vulnerability that could expose a system,
credential, private dataset, or enable unsafe robot motion. Use GitHub's
**Security → Report a vulnerability** private-reporting flow for this repository.

Include:

- the affected commit and files;
- reproduction steps using simulation where possible;
- the expected impact, including any physical safety impact;
- suggested mitigations, if known.

Do not test a vulnerability on hardware, accounts, networks, or data you do not
own or have explicit permission to use. Do not endanger people or equipment to
demonstrate an issue.

If private vulnerability reporting is unavailable, contact the repository
maintainer through their GitHub profile without publishing exploit details.

## Scope note

This policy covers the code in this repository. SteamVR, VIVE, MuJoCo, dora-rs,
OpenArm firmware, operating-system packages, and vendored upstream components
may have their own security processes; report upstream defects to their
maintainers as well.
