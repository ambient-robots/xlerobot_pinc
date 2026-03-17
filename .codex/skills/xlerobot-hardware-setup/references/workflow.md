# XLerobot Hardware Setup Reference

## Core files

- `README.md`
  Operator workflow: one-time setup, per-terminal steps, udev serial symlinks, motor ID setup, runtime JSON.
- `xlerobot_user_config.example.json`
  User-editable runtime schema for `has_mobile_platform`, `urdf_path`, and camera identifiers.
- `load_xlerobot_env.sh`
  JSON parsing, validation, and `XLEROBOT_*` env export behavior.
- `src/lerobot/robots/xlerobot_pinc/config_xlerobot_pinc.py`
  Env-backed config loading and validation for cameras and `has_mobile_platform`.
- `src/lerobot/robots/xlerobot_pinc/xlerobot_pinc.py`
  Robot-side hardware procedures: `connect()`, `calibrate()`, `configure()`, `setup_motors()`, optional mobile-base handling.

## Common tasks

### Add or change a runtime config key

Update these together:
- `xlerobot_user_config.example.json`
- `load_xlerobot_env.sh`
- `src/lerobot/robots/xlerobot_pinc/config_xlerobot_pinc.py`
- `README.md`

### Diagnose missing ports or board mapping

Check:
- README udev rule instructions
- config default ports in `config_xlerobot_pinc.py`
- whether the task is about board identity, not robot logic

### Diagnose calibration or setup flow

Inspect:
- `XLerobotPinc.connect()`
- `XLerobotPinc.calibrate()`
- `XLerobotPinc.configure()`
- `XLerobotPinc.setup_motors()`

Pay attention to:
- left bus contains arm plus optional base motors
- right bus contains right arm plus head motors
- `has_mobile_platform` gates base calibration, velocity mode, and stop behavior

## Useful checks

- `bash -n load_xlerobot_env.sh`
- `python -m compileall src examples/xlerobot_pinc`
- `rg -n "XLEROBOT_|has_mobile_platform" README.md src examples/xlerobot_pinc`
