# Bill of Materials (BOM)
 
## Servo Motors
 
| # | Component | Qty | Unit Price (€) | Total (€) | Link |
|---|-----------|-----|-----------------|-----------|------|
| 1 | STS3250 Servo (50kg.cm, 12V) | 6 | ~50.00 | ~300.00 | [AliExpress](https://de.aliexpress.com/item/1005010004108335.html) |
| 2 | STS3215 Servo (30kg.cm, 12V) | 10 | ~19.00 | ~190.00 | [AliExpress](https://de.aliexpress.com/item/1005009339011602.html) |
 
## [Optional] Motors for Platform  (See [XLerobot](https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/material.html#feature-parts))
 
| # | Component | Qty | Unit Price (€) | Total (€) | Link |
|---|-----------|-----|-----------------|-----------|------|
| 3 | STS215 Servo (30kg.cm, 12V) | 3 | ~19.00 | 57.00 | [AliExpress](https://de.aliexpress.com/item/1005009339011602.html) |
 
## Servo Control & Power
 
| # | Component | Qty | Unit Price (€) | Total (€) | Link |
|---|-----------|-----|-----------------|-----------|------|
| 4 | Waveshare Serial Bus Servo Driver (12V) | 2 | ~11.00 | ~22.00 | [Amazon.de](https://www.amazon.de/Waveshare-Integrates-Control-Applicable-Integrate/dp/B0CJ6TP3TP) |
| 5 | 12V Power Supply | 1 | 18.99 | 18.99 | [Amazon.de](https://www.amazon.de/Netzteil-100V-240V-Spannungswandler-Transformator-Stromversorgung/dp/B0FHNWXS27) |
| 6 | DC Splitter Cable (1-to-2, 90°) | 1 | 10.21 | 10.21 | [Amazon.de](https://www.amazon.de/GINTOOYUN-DC-Splitterkabel-Y-Splitter-Adapter-90-Grad-Stecker-Verlängerungskabel/dp/B09NPXTKYC) |
 
## Vision / Cameras
 
| # | Component | Qty | Unit Price (€) | Total (€) | Link | Comment|
|---|-----------|-----|-----------------|-----------|------|------|
| 7 | Intel RealSense D435i Depth Camera | 1 | ~589.00 | ~589.00 | [Amazon.de](https://www.amazon.de/Intel-RealSense-82635D435IDK5P-RealSense-Tiefen-Kamera-D435i/dp/B07MWR2YJB) ||
| 8 | Fish-Eye Lens Camera Module (168° horizontal FOV, USB-C 3.0, supports 640×480@60 Hz) | 2 | ~100.00 | ~200.00 | [AliExpress Store](https://www.aliexpress.com/store/4588015)/[Taobao (China)](https://item.taobao.com/item.htm?id=785049105422&mi_id=0000e4KUoVlTJSdhkV74JtTjLZJkbENbeQqvv3kuw0gurcw&skuId=5875965485322) |For the fish-eye option, contact the seller on AliExpress if purchasing from Taobao is not possible.|
 
## Cables & Connectors
 
| # | Component | Qty | Unit Price (€) | Total (€) | Link |
|---|-----------|-----|-----------------|-----------|------|
| 9 | USB-C to USB-A Cable (Anker, 1.8 m) | 2 | 3.43 | 6.86 | [Amazon.de](https://www.amazon.de/Anker-doppelt-geflochtenes-Ladekabel-Samsung-Schwarz/dp/B07DC5PPFV) |
| 10 | JST Wire Connector Kit (servo cable extension) | 1 | ~7.50 | ~7.50 | [Amazon.de](https://www.amazon.de/Männlich-Weiblich-Adapterkabel-Verbinder-Platinenstecker/dp/B0BZHR5NCR) |
| 11 | 3-Pin Extension Cable (servo) | 1 | ~5.50 | ~5.50 | [Amazon.de](https://www.amazon.de/gp/product/B0F88H5DLC) |
 
---
 


### Grippers 

The BOM for the grippers can be found [here](https://github.com/pollen-robotics/PincOpen?tab=readme-ov-file#bom-bill-of-materials) 
> **Note:** the Servo and control board are already included in the upper BOM

---
## Summary
 
| Category | Subtotal (€) |
|----------|---------------|
| Servo Motors | ~490.00 |
| Servo Control & Power | ~51.20 |
| Vision / Cameras | ~789.00 |
| Cables & Connectors | ~19.86 |
|2x[PincOpen Gripper](https://github.com/pollen-robotics/PincOpen?tab=readme-ov-file#bom-bill-of-materials)|~52,92€|
| **Grand Total** | **~1,402.98** |
 
> **Note:** Prices marked with ~ are approximate and may vary. The platform is not included in this price, please check the [Xlerobot](https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/material.html) for that 
 

# How to use it
## Quick Start
One-time setup:
1. Link this repo into your `lerobot` checkout (see [Link xlerobot_pro into lerobot](#link-xlerobot-into-lerobot)).
2. Set up USB serial symlinks for motor boards (see [Set up USB serial symlinks for motor boards](#usb-serial-symlinks)). This is required so default robot ports resolve correctly: `/dev/xlerobot_right_head` and `/dev/xlerobot_left_base`.
3. Configure motor ids (see [Configure motor ids](#configure-motor-ids)).
4. Create and edit runtime config (see [User Runtime Config (JSON -> Env Vars)](#user-runtime-config)).

For each new terminal:
1. `conda activate lerobot`
2. `source /path/to/xlerobot_pro/load_xlerobot_env.sh`
3. Run scripts from `/path/to/xlerobot_pro/examples/xlerobot_pro/`.

Example run command:
```bash
python /path/to/xlerobot_pro/examples/xlerobot_pro/4_xlerobot_pro_teleop_keyboard.py
```

## Hardware
Physical hardware lineage and source notes are documented under [hardware/README.md](hardware/README.md).

This repo is extended from `XLeRobot`, upgraded to an SO107-style 6-DoF arm setup, and uses a gripper based on `PincOpen`. The `hardware/` folder documents where to obtain the relevant upstream printable files.

<a id="link-xlerobot-into-lerobot"></a>
## Link xlerobot_pro into lerobot
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

If a destination path already exists in `lerobot` as a normal file/directory (not a symlink), the script moves it to a timestamped backup like `*.bak.YYYYMMDD_HHMMSS` before creating the new symlink.

<a id="usb-serial-symlinks"></a>
## Set up USB serial symlinks for motor boards
This step is required when using the default ports in `XLerobotProConfig`:
- `/dev/xlerobot_right_head`
- `/dev/xlerobot_left_base`

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
ls -l /dev/xlerobot_right_head /dev/xlerobot_left_base
```

Expected output shape:
```
/dev/xlerobot_right_head -> ttyACMx
/dev/xlerobot_left_base -> ttyACMy
```

<a id="configure-motor-ids"></a>
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

<a id="user-runtime-config"></a>
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

Required camera keys:
- `cameras.left_wrist.index_or_path`
- `cameras.right_wrist.index_or_path`
- `cameras.head.serial_number_or_name`

Optional camera tuning keys:
- `fps`, `width`, `height` for each camera
- `fourcc` for `left_wrist` and `right_wrist` (OpenCV cameras only)

Load it in each shell before running scripts:
```bash
source /path/to/xlerobot_pro/load_xlerobot_env.sh
```

`load_xlerobot_env.sh` exports:
- global:
  - `XLEROBOT_HAS_MOBILE_PLATFORM`
  - `XLEROBOT_URDF_PATH`
- left_wrist:
  - `XLEROBOT_LEFT_WRIST_INDEX_OR_PATH`
  - `XLEROBOT_LEFT_WRIST_FPS` (optional)
  - `XLEROBOT_LEFT_WRIST_WIDTH` (optional)
  - `XLEROBOT_LEFT_WRIST_HEIGHT` (optional)
  - `XLEROBOT_LEFT_WRIST_FOURCC` (optional)
- right_wrist:
  - `XLEROBOT_RIGHT_WRIST_INDEX_OR_PATH`
  - `XLEROBOT_RIGHT_WRIST_FPS` (optional)
  - `XLEROBOT_RIGHT_WRIST_WIDTH` (optional)
  - `XLEROBOT_RIGHT_WRIST_HEIGHT` (optional)
  - `XLEROBOT_RIGHT_WRIST_FOURCC` (optional)
- head:
  - `XLEROBOT_HEAD_SERIAL_NUMBER_OR_NAME`
  - `XLEROBOT_HEAD_FPS` (optional)
  - `XLEROBOT_HEAD_WIDTH` (optional)
  - `XLEROBOT_HEAD_HEIGHT` (optional)

All xlerobot_pro examples and config now read these values, so no script-by-script edits are required for camera paths/serials.

## Acknowledgements

`xlerobot_pro` is extended from [XLeRobot](https://github.com/Vector-Wangel/XLeRobot), and both the robot platform and parts of the software stack in this repo build on that upstream project. This includes adapted software components derived from that lineage, such as the bundled `XLeVR` VR teleoperation module.

The gripper setup used in the SO107-style arm configuration is based on [PincOpen](https://github.com/pollen-robotics/PincOpen).

See [hardware/README.md](hardware/README.md) for hardware source notes.
