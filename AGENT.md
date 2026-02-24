# AGENT.md

Guidelines for coding agents working in this repository.

## Scope
- Repository: `xlerobot_pro`
- Main ownership area: `src/lerobot/robots/xlerobot_pro/` and `examples/xlerobot_pro/`
- Goal: keep the `xlerobot_pro` integration clean, hardware-aware, and easy to sync into a main `lerobot` checkout.

## Architectural Rules
- Use one unified robot implementation: `XLerobotPro` + `XLerobotProConfig`.
- Do not reintroduce `xlerobot_pro_tah` split classes/configs.
- Backward compatibility is not required for deprecated names in this experimental repo.
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

## Example Script Rules
- Prefer unified examples under `examples/xlerobot_pro/`.
- For scripts that use base motion, gate behavior on robot/config `has_mobile_platform`.
- Avoid script-specific constants that must be manually edited by each user.
- If example behavior changes, update README usage snippets in the same change.

## Symlink Workflow Rules
- Use `setup_lerobot_symlinks.sh` (repo root) for linking into a `lerobot` checkout.
- Prefer symlink workflow over file copying.
- Keep script defaults relative/generic; avoid hardcoded user home paths.

## Code Quality Rules
- Keep changes minimal and targeted.
- Avoid broad refactors unrelated to requested behavior.
- Preserve existing style unless there is a clear correctness/readability issue.
- Add comments only where logic is non-obvious.
- Do not silently change hardware semantics (motor IDs, calibration behavior, limits) without explicit request.

## Validation Checklist
- Run targeted static checks after edits:
  - `python -m compileall src examples/xlerobot_pro`
  - `bash -n load_xlerobot_env.sh setup_lerobot_symlinks.sh`
- Verify grep-level invariants:
  - no stale `xlerobot_pro_tah` references
  - no new hardcoded user-specific absolute paths in docs/examples
- Check `git status` and review diffs before concluding.

## Documentation Rules
- Keep `README.md` consistent with current scripts and file locations.
- Use generic placeholders like `/path/to/...` in docs.
- Document any new env vars, required config fields, or workflow changes.

## Safety Rules
- Never run destructive git commands (`reset --hard`, checkout file reverts) unless explicitly requested.
- Do not revert unrelated user changes in a dirty worktree.
- If unexpected modifications appear mid-task, stop and ask how to proceed.
