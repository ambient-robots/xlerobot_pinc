# XLerobot LeRobot Sync Reference

## Script under control

- `setup_lerobot_symlinks.sh`

## Paths currently linked into `lerobot`

- `src/lerobot/robots/xlerobot_pro`
- `src/lerobot/robots/so107_follower`
- `src/lerobot/model/kinematics.py`
- `src/lerobot/utils/quadratic_spline_via_ipol.py`
- `src/lerobot/scripts/motor_id_tool.py`

## Paths intentionally not linked

- `examples/`
- `XLeVR/`
- root helper files such as `load_xlerobot_env.sh`

## Verification checklist

After changing the sync script:
- run `bash -n setup_lerobot_symlinks.sh`
- verify default path resolution is still relative/generic
- verify backup behavior still uses `*.bak.<timestamp>`
- verify README linked-path inventory still matches the script

## When not to use this skill

Do not use this skill for runtime config or hardware calibration changes unless the task also affects the sync/bootstrap workflow.
