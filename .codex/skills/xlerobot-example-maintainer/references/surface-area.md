# XLerobot Example Maintainer Reference

## Main example files

- `examples/xlerobot_pinc/4_xlerobot_pinc_teleop_keyboard.py`
  Keyboard teleop, head control, optional base behavior, URDF env usage.
- `examples/xlerobot_pinc/9_vr_teleop_xlerobot_pinc.py`
  Single-robot VR flow with optional base handling.
- `examples/xlerobot_pinc/9_vr_teleop_xlerobot_pinc_dualarm.py`
  Dual-arm VR teleop flow.
- `examples/xlerobot_pinc/9_vr_teleop_xlerobot_pinc_dualarm_w_dataset_recording.py`
  Dual-arm VR teleop plus dataset recording.
- `examples/xlerobot_pinc/9_pi05_inference_xlerobot_pinc_dualarm.py`
  Inference-oriented example using the same robot/config surface.

## Shared dependencies

- `src/lerobot/robots/xlerobot_pinc/config_xlerobot_pinc.py`
- `src/lerobot/robots/xlerobot_pinc/xlerobot_pinc.py`
- `load_xlerobot_env.sh`
- `xlerobot_user_config.example.json`
- `README.md`

## Coordinated changes to watch for

### Config or env var changes

Update:
- config consumer
- env loader
- JSON template
- every affected example
- README

### Base/no-base behavior changes

Check:
- `robot.has_mobile_platform` gating in examples
- `.vel` action usage in the robot class
- any example-level flags such as `ENABLE_BASE`

### URDF or camera setup changes

Check:
- `XLEROBOT_URDF_PATH` usage
- example startup assumptions
- README operator instructions
