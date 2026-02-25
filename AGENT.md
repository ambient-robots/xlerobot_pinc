# AGENT.md

Guidelines for coding agents working in this repository.

## Scope
- Repository: `xlerobot_pro`
- Main ownership area: `src/lerobot/robots/xlerobot_pro/` and `examples/xlerobot_pro/`
- Goal: keep the `xlerobot_pro` integration clean, hardware-aware, and easy to sync into a main `lerobot` checkout.

## Architectural Rules
- Use one unified robot implementation: `XLerobotPro` + `XLerobotProConfig`.
- Treat mobile base support as configuration-driven.
  - `has_mobile_platform` determines whether base motors/features are active.
  - Base-specific logic must be conditional in public methods like `connect()`, `calibrate()`, and `configure()`.

## Config and Environment Rules
- `has_mobile_platform` must be provided by either:
  - explicit config value, or
  - `XLEROBOT_HAS_MOBILE_PLATFORM` env var.
- Keep `xlerobot_user_config.example.json` as the editable template for users.
- Keep `load_xlerobot_env.sh` in repo root and aligned with JSON keys.
- Use `XLEROBOT_URDF_PATH` for URDF path injection in examples instead of hardcoded user paths.
- Camera runtime config schema is nested under `cameras` in user JSON:
  - required: `cameras.left_wrist.index_or_path`, `cameras.right_wrist.index_or_path`, `cameras.head.serial_number_or_name`
  - optional: per-camera `fps/width/height`, plus `fourcc` for OpenCV wrist cameras
- `load_xlerobot_env.sh` should be the single translation layer from JSON to env vars; avoid duplicated parsing logic in examples.
- For scripts that are sourced (`load_xlerobot_env.sh`), never mutate caller shell options globally (avoid `set -euo pipefail` side effects).
- Fail fast with explicit validation errors for missing/invalid required config keys.

## Example Script Rules
- Prefer unified examples under `examples/xlerobot_pro/`.
- For scripts that use base motion, gate behavior on robot/config `has_mobile_platform`.
- Avoid script-specific constants that must be manually edited by each user.
- If example behavior changes, update README usage snippets in the same change.

## Symlink Workflow Rules
- Use `setup_lerobot_symlinks.sh` (repo root) for linking into a `lerobot` checkout.
- Prefer symlink workflow over file copying.
- Keep script defaults relative/generic; avoid hardcoded user home paths.
- `setup_lerobot_symlinks.sh` links core integration files only; do not link `examples/` or `XLeVR/`.
- If destination paths already exist and are not symlinks, keep backup-before-link behavior (`.bak.<timestamp>`).

## Code Quality Rules
- Keep changes minimal and targeted.
- Avoid broad refactors unrelated to requested behavior.
- Preserve existing style unless there is a clear correctness/readability issue.
- Add comments only where logic is non-obvious.
- Do not silently change hardware semantics (motor IDs, calibration behavior, limits) without explicit request.
- Prefer multiple focused commits grouped by logical concern over a single large mixed commit.
- When changing config surface area, update all affected layers in the same iteration:
  1. config consumer (`config_xlerobot_pro.py`)
  2. env loader (`load_xlerobot_env.sh`)
  3. template (`xlerobot_user_config.example.json`)
  4. docs (`README.md`)

## Validation Checklist
- Run targeted static checks after edits:
  - `python -m compileall src examples/xlerobot_pro`
  - `bash -n load_xlerobot_env.sh setup_lerobot_symlinks.sh`
- Verify grep-level invariants:
  - no new hardcoded user-specific absolute paths in docs/examples
- Check `git status` and review diffs before concluding.

## Documentation Rules
- Keep `README.md` consistent with current scripts and file locations.
- Use generic placeholders like `/path/to/...` in docs.
- Document any new env vars, required config fields, or workflow changes.
- Maintain a clear "one-time setup" vs "per-terminal steps" workflow.
- In workflow docs, explicitly include motor board udev symlink setup before running robot scripts because default ports depend on `/dev/xlerobot_right_head` and `/dev/xlerobot_left_base`.
- Keep `source load_xlerobot_env.sh` output concise and operator-friendly (one summary line per camera is preferred).

## Iteration Methodology
- Use small, testable increments: implement -> validate -> sync docs -> revalidate.
- For each behavior change, verify both positive and negative paths (e.g., valid config loads, invalid config fails with actionable error).
- Before committing, re-check `git diff` for accidental scope creep and split commits by concern (runtime/config vs docs/workflow).

## Safety Rules
- Never run destructive git commands (`reset --hard`, checkout file reverts) unless explicitly requested.
- Do not revert unrelated user changes in a dirty worktree.
- If unexpected modifications appear mid-task, stop and ask how to proceed.
