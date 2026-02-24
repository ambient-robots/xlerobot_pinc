## Initial Setup
Use symlinks instead of copying files manually.

From this repository:
```bash
cd /path/to/xlerobot_pro
./examples/xlerobot_pro/setup_lerobot_symlinks.sh /path/to/lerobot
```

This links robot sources, examples, and `XLeVR` into your main `lerobot` checkout and avoids repetitive copy/paste sync mistakes.

## User Runtime Config (JSON -> Env Vars)
Create your local user config:
```bash
cp xlerobot_user_config.example.json xlerobot_user_config.json
```

Edit `xlerobot_user_config.json`:
```json
{
  "has_mobile_platform": false,
  "urdf_path": "/absolute/path/to/ambient_urdf/robot.urdf"
}
```

Load it in each shell before running scripts:
```bash
source /path/to/xlerobot_pro/load_xlerobot_env.sh
```

This exports:
- `XLEROBOT_HAS_MOBILE_PLATFORM`
- `XLEROBOT_URDF_PATH`

All xlerobot_pro examples and config now read these values, so no script-by-script edits are required.

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
