# XLerobot Example Maintainer Reference

## Main example files

- `examples/xlerobot_pinc/keyboard_teleop_full_body.py`
  Keyboard teleop, head control, optional base behavior, URDF env usage.
- `examples/xlerobot_pinc/vr_teleop_full_body.py`
  Full-body VR flow with optional head and base handling.
- `examples/xlerobot_pinc/vr_teleop_dualarm.py`
  Dual-arm VR teleop flow.
- `examples/xlerobot_pinc/vr_teleop_dualarm_dataset.py`
  Dual-arm VR teleop plus dataset recording.
- `examples/xlerobot_pinc/pi05_inference_dualarm.py`
  Inference-oriented example using the same robot/config surface.

## Standalone debug helpers

- `examples/so107_follower/keyboard_joint_control.py`
  Single-arm joint-space keyboard teleop for SO107 debugging.
- `examples/so107_follower/keyboard_ee_control.py`
  Single-arm end-effector keyboard teleop using `XLEROBOT_URDF_PATH`.

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
- the standalone SO107 debug helpers if the change affects shared URDF handling
