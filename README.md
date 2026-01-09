## Initial Setup
Clone the official `higgingface/lerobot` and follow the instructions in https://huggingface.co/docs/lerobot/installation to install lerobot from source.

Copy the robot folders into `src/lerobot/robots/` in your local clone of `higgingface/lerobot`
- For `so107_follower` (6-DoF arm with a pinc open gripper), copy the folder: `src/lerobot/robots/so107_follower`
- For `xlerobot_pro_tah` (bi-manual so107 with head pan–tilt) and `xlerobot_pro` (with additionally the lekiwi base), copy the folder: `src/lerobot/robots/xlerobot_pro`

To use VR teleoperation, copy the XLeVR folder into the same directory where the [vr teleop script](https://github.com/xuweiwu/lerobot_add_on/blob/main/examples/xlerobot_pro/9_vr_teleop_xlerobot_pro_tah.py) is located. Then update `XLEVR_PATH` in [`vr_monitor.py`](https://github.com/xuweiwu/lerobot_add_on/blob/dfc49a0324ec4249d0998b09bc4360295f8beef0/XLeVR/vr_monitor.py#L21)  accordingly.

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
from lerobot.robots.xlerobot_pro import XLerobotProTAH, XLerobotProTAHConfig

config = XLerobotProTAHConfig(
    id="ambient_xlerobot_pro_tah",
    use_degrees=True
)
robot = XLerobotProTAH(config)
robot.setup_motors()
```
