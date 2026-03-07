---
name: xlerobot-example-maintainer
description: Update and verify the example scripts under `examples/xlerobot_pro/` when robot config, environment variables, optional mobile-base behavior, URDF handling, or camera/runtime assumptions change. Use when Codex needs to keep the examples consistent with `XLerobotPro`, `XLerobotProConfig`, `load_xlerobot_env.sh`, or the user runtime JSON schema.
---

# Xlerobot Example Maintainer

## Overview

Use this skill for coordinated maintenance of the example scripts. Focus on consistency between example behavior, the unified robot/config implementation, and the env-backed runtime setup.

## Workflow

1. Read the relevant example scripts in `examples/xlerobot_pro/`.
2. Read `src/lerobot/robots/xlerobot_pro/config_xlerobot_pro.py` and `src/lerobot/robots/xlerobot_pro/xlerobot_pro.py`.
3. Read `load_xlerobot_env.sh` and `xlerobot_user_config.example.json` when the task touches env-driven behavior.
4. Update every affected example in the same iteration when a shared assumption changes.
5. Update `README.md` if the operator-facing workflow changes.

## Operating Rules

- Gate base-only behavior on `robot.has_mobile_platform` or the corresponding config state.
- Avoid script-specific user path edits; prefer env-driven values such as `XLEROBOT_URDF_PATH`.
- Keep example naming and scope unified under `examples/xlerobot_pro/`.
- Prefer removing obsolete duplicated examples when a unified example fully replaces them.
- Do not change control semantics casually in one script while leaving sibling examples inconsistent.

## High-Risk Surfaces

- `has_mobile_platform` changes affect both robot actions and example control paths.
- `XLEROBOT_*` env changes can break example startup silently if only some scripts are updated.
- VR examples contain duplicated control scaffolding; check all related files before concluding a task is done.

## Validation

- Run `python -m compileall src examples/xlerobot_pro`.
- Grep for env vars or config keys touched by the change across all examples.
- Re-read `README.md` if the example invocation or setup assumptions changed.

## References

- Read `references/surface-area.md` for file grouping and common coordinated-change patterns.
