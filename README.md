## Initial Setup
Use symlinks instead of copying files manually.

From this repository:
```bash
cd /path/to/xlerobot_pro
./setup_lerobot_symlinks.sh /path/to/lerobot
```

This links the following paths into your main `lerobot` checkout:
- `src/lerobot/robots/xlerobot_pro`
- `src/lerobot/robots/so107_follower`
- `src/lerobot/model/kinematics.py`
- `src/lerobot/utils/quadratic_spline_via_ipol.py`
- `src/lerobot/scripts/motor_id_tool.py`
It intentionally does not link `examples/` or `XLeVR/`.

Sidenote: if a destination path already exists in `lerobot` as a normal file/directory (not a symlink), the script moves it to a timestamped backup like `*.bak.YYYYMMDD_HHMMSS` before creating the new symlink.

## Recommended workflow
One-time setup:
1. Run `./setup_lerobot_symlinks.sh /path/to/lerobot`.
2. Set up USB serial symlinks for motor boards (follow `Set up symbolic links to USB serial devices` below). This is required so default robot ports resolve correctly: `/dev/xlerobot_right_head` and `/dev/xlerobot_left_base`.
3. Verify symlinks exist: `ls -l /dev/xlerobot_right_head /dev/xlerobot_left_base`.
4. Create local config: `cp xlerobot_user_config.example.json xlerobot_user_config.json`.
5. Edit `xlerobot_user_config.json` with your own camera and URDF paths.

For each new terminal:
1. `conda activate lerobot`
2. `source /path/to/xlerobot_pro/load_xlerobot_env.sh`
3. Run scripts.

Example run command:
```bash
python /path/to/xlerobot_pro/examples/xlerobot_pro/4_xlerobot_pro_teleop_keyboard.py
```
Use the same pattern for other scripts under `/path/to/xlerobot_pro/examples/xlerobot_pro/`.

## User Runtime Config (JSON -> Env Vars)
Create your local user config:
```bash
cp xlerobot_user_config.example.json xlerobot_user_config.json
```

Edit `xlerobot_user_config.json`:
```json
{
  "has_mobile_platform": false,
  "urdf_path": "/absolute/path/to/so107_urdf",
  "cameras": {
    "left_wrist": {
      "index_or_path": "/dev/videoX or /dev/v4l/by-path/platform-XXXX-left-video-index0",
      "fps": 60,
      "width": 640,
      "height": 480,
      "fourcc": "YUYV"
    },
    "right_wrist": {
      "index_or_path": "/dev/videoX or /dev/v4l/by-path/platform-XXXX-right-video-index0",
      "fps": 60,
      "width": 640,
      "height": 480,
      "fourcc": "YUYV"
    },
    "head": {
      "serial_number_or_name": "serial number of your realsense head camera",
      "fps": 60,
      "width": 640,
      "height": 480
    }
  }
}
```
`cameras.left_wrist.index_or_path`, `cameras.right_wrist.index_or_path`, and `cameras.head.serial_number_or_name` are required.
`fps/width/height/fourcc` keys are optional overrides (defaults are currently 60/640/480 and `YUYV` for wrist cameras). `fourcc` applies only to OpenCV wrist cameras.
The nested `cameras` structure is required by `load_xlerobot_env.sh`.

Load it in each shell before running scripts:
```bash
source /path/to/xlerobot_pro/load_xlerobot_env.sh
```

This exports:
- `XLEROBOT_HAS_MOBILE_PLATFORM`
- `XLEROBOT_URDF_PATH`
- `XLEROBOT_LEFT_WRIST_INDEX_OR_PATH`
- `XLEROBOT_RIGHT_WRIST_INDEX_OR_PATH`
- `XLEROBOT_HEAD_SERIAL_NUMBER_OR_NAME`
- `XLEROBOT_LEFT_WRIST_FPS` (optional)
- `XLEROBOT_LEFT_WRIST_WIDTH` (optional)
- `XLEROBOT_LEFT_WRIST_HEIGHT` (optional)
- `XLEROBOT_LEFT_WRIST_FOURCC` (optional)
- `XLEROBOT_RIGHT_WRIST_FPS` (optional)
- `XLEROBOT_RIGHT_WRIST_WIDTH` (optional)
- `XLEROBOT_RIGHT_WRIST_HEIGHT` (optional)
- `XLEROBOT_RIGHT_WRIST_FOURCC` (optional)
- `XLEROBOT_HEAD_FPS` (optional)
- `XLEROBOT_HEAD_WIDTH` (optional)
- `XLEROBOT_HEAD_HEIGHT` (optional)

All xlerobot_pro examples and config now read these values, so no script-by-script edits are required for camera paths/serials.

## Set up symbolic links to USB serial devices (aka motor servo boards)
The advantage is the actual `/dev/ttyACM0` of a motor servo board may change, but the symlink stays valid.

### 0. Find the USB port associated with each arm
Use `lerobot-find-port` to find the currently assigned port for the device, as shown in https://huggingface.co/docs/lerobot/so101#1-find-the-usb-ports-associated-with-each-arm .

### 1. Identify the device uniquely
Plug in the device and run (replace `/dev/ttyACM0` with the port you found in the previous step):
```
udevadm info -a -n /dev/ttyACM0
```
You will see a long output. Look for stable attributes, typically:
- idVendor
- idProduct
- serial
Example (yours will differ):
```
ATTRS{idVendor}=="2341"
ATTRS{idProduct}=="0043"
ATTRS{serial}=="85735313333351E0F1A1"
```
### 2. Create a udev rule
Create a custom rule file:
```
sudo vim /etc/udev/rules.d/99-xlerobot-serial.rules
```
Example rule:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5AAF288029", MODE="0777", SYMLINK+="xlerobot_right_head"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5A7C121362", MODE="0777", SYMLINK+="xlerobot_left_base"
```
This creates two symbolic links to the motor servo boards.

### 3. Reload udev rules
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Or simply unplug and replug the device.

### 4. Verify
```
ls -l /dev/xlerobot_left_base
```
Expected output:
```
/dev/xlerobot_left_base -> ttyACM1
```

## Configure motor ids
Follow the steps in https://huggingface.co/docs/lerobot/so101#follower to configure the motor ids.

### Single `so107_follower` arm
For a single `so107_follower` arm, run:
```
lerobot-setup-motors \
    --robot.type=so107_follower \
    --robot.port=/dev/xlerobot_right_head
```
If this arm will be used as the left arm in a bimanual setup, change `--robot.port` to `/dev/xlerobot_left_base`.

### Complete bimanual + pan-tilt setup
For the complete bimanual + pan-tilt setup, configuring all motors via a command-line option is not supported. Instead, use the API:
```
from lerobot.robots.xlerobot_pro import XLerobotPro, XLerobotProConfig

config = XLerobotProConfig(
    id="ambient_xlerobot_pro",
    has_mobile_platform=True,
    use_degrees=True
)
robot = XLerobotPro(config)
robot.setup_motors()
```

If you prefer env-based setup, you can omit `has_mobile_platform` and set `XLEROBOT_HAS_MOBILE_PLATFORM` instead.
