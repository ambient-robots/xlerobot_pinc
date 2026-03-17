# AGENTS.md

Repository-level instructions for coding agents working in `xlerobot_pinc`.

## Scope
- Primary areas: `src/lerobot/robots/xlerobot_pinc/`, `examples/xlerobot_pinc/`, root workflow scripts, and `README.md`.
- Treat this file as always-on policy. Put deeper task-specific procedures in skills, not here.

## Architecture
- Keep `xlerobot_pinc` as the unified robot integration entrypoint: `XLerobotPinc` and `XLerobotPincConfig`.
- Treat mobile-base support as configuration-driven through `has_mobile_platform`.
- When changing behavior that differs between base and no-base setups, update the robot class and affected examples consistently.

## Config Surface
- Keep runtime configuration coherent across these layers:
  1. `src/lerobot/robots/xlerobot_pinc/config_xlerobot_pinc.py`
  2. `load_xlerobot_env.sh`
  3. `xlerobot_user_config.example.json`
  4. `README.md`
- Prefer one translation path from user config to runtime settings. Avoid duplicating config parsing logic across scripts.
- Fail fast with explicit validation errors for missing or invalid required config.

## Example Scripts
- Prefer unified examples under `examples/xlerobot_pinc/`.
- Base-specific teleop or motion logic must be gated on robot/config state, not assumed unconditionally.
- Avoid hardcoded user-specific paths in examples.

## Tooling And Sync
- Prefer the symlink workflow over manual copying into a separate `lerobot` checkout.
- Keep repo-root helper scripts generic and portable.

## Change Discipline
- Keep changes minimal and scoped to the task.
- Preserve existing style unless there is a clear correctness or maintainability issue.
- Do not silently change hardware semantics such as motor IDs, calibration behavior, control modes, or safety limits without explicit intent.
- Prefer multiple focused commits grouped by logical concern over a single large mixed commit.

## Validation
- After edits, run targeted checks relevant to the touched files.
- For Python and shell changes, default checks are:
  - `python -m compileall src examples/xlerobot_pinc`
  - `bash -n load_xlerobot_env.sh setup_lerobot_symlinks.sh`
- Review `git diff` for accidental scope creep before concluding.

## Documentation
- Keep `README.md` aligned with the current workflow and file locations.
- Use generic placeholders like `/path/to/...` in docs.
- Document user-visible config, environment, and setup changes in the same iteration as the code change.

## Safety
- Never use destructive git commands unless explicitly requested.
- Do not revert unrelated user changes in a dirty worktree.
- If unexpected changes conflict directly with the current task, stop and ask how to proceed.
