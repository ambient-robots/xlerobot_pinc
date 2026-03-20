# XLeVR

This directory contains the `xlerobot_pinc` fork of XLeVR, the lightweight VR input layer used by the VR teleoperation examples in this repository.

It is based on the upstream XLeVR module from XLeRobot, but it is not a one-to-one copy. The main local changes are made so it works with the SO107-style arms and the Placo-based IK pipeline used in this repo.

## Main difference from upstream

Upstream XLeVR is oriented around the original XLeRobot arm control path, where the VR layer can provide arm targets tailored to an analytical IK setup for the 5-DoF arm. In that model, the control message includes values such as:

- `target_position`
- `wrist_roll_deg`
- `wrist_flex_deg`
- `gripper_closed`

This fork does not use that contract.

For `xlerobot_pinc`, VR input is treated as a relative pose command that is consumed later by the teleoperation code and converted to robot actions through the Placo IK solver. This also aligns with upstream [LeRobot](https://github.com/huggingface/lerobot), where the default `RobotKinematics` IK helper is implemented on top of `placo`.

Instead of upstream wrist-angle fields, this fork sends:

- `mode`
- `relative_position`
- `relative_rotvec`
- `trigger`
- `thumbstick`
- `buttons`
- `vr_ctrl_rotation`

It also adds a `RESET` mode so the downstream teleop code can calibrate the VR controller frame against the robot end-effector frame before normal motion updates begin.

## How it is used in this repo

In this repository, XLeVR is an input frontend, not the final arm solver.

The overall flow is:

1. The browser-side WebXR app sends controller pose and button data over WebSocket.
2. `XLeVR/xlevr/inputs/vr_ws_server.py` converts that into `ControlGoal` messages with relative position and orientation updates.
3. `XLeVR/vr_monitor.py` queues those messages per arm and exposes them to the teleoperation loop.
4. The `xlerobot_pinc` VR examples consume those goals and run the downstream IK pipeline through LeRobot's default `placo`-based kinematics layer.

See for example:

- [`examples/xlerobot_pinc/9_vr_teleop_xlerobot_pinc_dualarm.py`](../examples/xlerobot_pinc/9_vr_teleop_xlerobot_pinc_dualarm.py)
- [`examples/xlerobot_pinc/9_vr_teleop_xlerobot_pinc.py`](../examples/xlerobot_pinc/9_vr_teleop_xlerobot_pinc.py)
- [`src/lerobot/model/kinematics.py`](../src/lerobot/model/kinematics.py)

## ControlGoal fields

The local `ControlGoal` dataclass is defined in [`xlevr/inputs/base.py`](xlevr/inputs/base.py).

Main fields:

- `arm`: target side, currently `"left"` or `"right"`
- `mode`: `POSITION_CONTROL`, `IDLE`, or `RESET`
- `relative_position`: controller translation delta in VR coordinates
- `relative_rotvec`: controller orientation delta as a rotation vector
- `trigger`: trigger state/value
- `thumbstick`: thumbstick state
- `buttons`: button state
- `vr_ctrl_rotation`: controller rotation captured at reset time
- `metadata`: extra debug or transport information

In other words, this fork exports VR motion intent, not already-solved arm joint targets.

## Installation

Install the XLeVR-specific dependencies:

```bash
pip install -r requirements.txt
```

## Running the monitor directly

From the `XLeVR/` directory:

```bash
python vr_monitor.py
```

Then open the HTTPS address shown in the terminal in the VR headset browser.

Notes:

- If you move or copy `vr_monitor.py`, update the `XLEVR_PATH` constant at the top of the file.
- In this repository, the more common usage is to import `VRMonitor` from the teleoperation examples rather than run it as a standalone utility.

## Typical integration pattern

The VR examples in this repository use `VRMonitor` roughly like this:

```python
vr_monitor = VRMonitor()
vr_monitor.initialize()

# start the async monitor in a background thread

left_reset, left_motion = vr_monitor.pop_ordered_goals_for_arm("left")
right_reset, right_motion = vr_monitor.pop_ordered_goals_for_arm("right")
```

The teleop loop then handles:

- `RESET` first, to align controller and end-effector frames
- `POSITION_CONTROL` updates next, as relative pose increments
- `IDLE` to stop or disengage motion updates

## Scope

This README describes the local XLeVR fork as used by `xlerobot_pinc`. For the original upstream behavior and assumptions, refer to the XLeRobot repository.
