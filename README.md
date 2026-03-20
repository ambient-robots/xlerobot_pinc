<p align="center">
  <img src="assets/XLerobot_pinc.svg" alt="XLeRobot Pinc" width="400"/>
</p>

<p align="center">
  <a href="https://github.com/Vector-Wangel/XLeRobot"><img src="https://img.shields.io/badge/XLeRobot-upstream-blue?style=for-the-badge" alt="XLeRobot"/></a>
  <a href="https://github.com/pollen-robotics/PincOpen"><img src="https://img.shields.io/badge/PincOpen-gripper-green?style=for-the-badge" alt="PincOpen"/></a>
  <a href="https://github.com/huggingface/lerobot"><img src="https://img.shields.io/badge/SO107-arm-orange?style=for-the-badge" alt="SO107"/></a>
</p>

---

# XLeRobot Pinc

A bimanual robot platform with dual SO107-style 6-DoF arms, PincOpen grippers, a pan-tilt head with depth camera, and optional omni-mobile base. Built on top of [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) and integrates with [LeRobot](https://github.com/huggingface/lerobot) for teleoperation, dataset recording, and policy inference.

## Quick Start

**One-time setup:**

1. [Link this repo into your `lerobot` checkout](#link-xlerobot_pinc-into-lerobot)
2. [Set up USB serial symlinks](#usb-serial-symlinks) so `/dev/xlerobot_right_head` and `/dev/xlerobot_left_base` resolve correctly
3. [Configure motor IDs](#configure-motor-ids)
4. [Create your runtime config](#user-runtime-config)

**Per terminal:**

```bash
conda activate lerobot
source /path/to/xlerobot_pinc/load_xlerobot_env.sh
python examples/xlerobot_pinc/4_xlerobot_pinc_teleop_keyboard.py
```

## Repository Structure

```
xlerobot_pinc/
├── src/lerobot/robots/xlerobot_pinc/   # Robot class & config
├── src/lerobot/robots/so107_follower/  # SO107 follower arm
├── examples/xlerobot_pinc/             # Teleop, VR, inference scripts
├── xlerobot_pinc_urdf/                       # Bundled URDF model & 3D assets
├── XLeVR/                              # VR teleoperation module
├── hardware/                           # Hardware docs, source refs, custom part exports
├── setup_lerobot_symlinks.sh           # Symlink installer for lerobot
├── load_xlerobot_env.sh                # Env loader from JSON config
└── xlerobot_user_config.example.json   # User config template
```

## Hardware

This platform extends [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) with SO107-style 6-DoF arms and [PincOpen](https://github.com/pollen-robotics/PincOpen) grippers. See [hardware/README.md](hardware/README.md) for hardware lineage and local custom parts, and [hardware/SOURCES.md](hardware/SOURCES.md) for upstream, Onshape, and local asset references.

The URDF models are bundled in [`xlerobot_pinc_urdf/`](xlerobot_pinc_urdf/) and can be referenced via the `urdf_path` config key:

- **`robot.urdf`** — single arm (6-DOF), used for inverse kinematics
- **`gripper.urdf`** — PincOpen end-effector (1-DOF cam)
- **`xlerobot.urdf`** — mobile base platform with dual arm mounts

All three are assembled together for full robot visualization; only `robot.urdf` is used for IK. Run `gen_local_paths.sh` inside the URDF directory to generate local-path variants.

<details>
<summary>Bill of Materials (BOM)</summary>

### Servo Motors

| # | Component | Qty | Unit Price (EUR) | Total (EUR) | Link |
|---|-----------|-----|-----------------|-----------|------|
| 1 | STS3250 Servo (50kg.cm, 12V) | 6 | ~50.00 | ~300.00 | [AliExpress](https://de.aliexpress.com/item/1005010004108335.html) |
| 2 | STS3215 Servo (30kg.cm, 12V) | 10 | ~19.00 | ~190.00 | [AliExpress](https://de.aliexpress.com/item/1005009339011602.html) |

#### [Optional] Motors for Platform (See [XLerobot](https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/material.html#feature-parts))

| # | Component | Qty | Unit Price (EUR) | Total (EUR) | Link |
|---|-----------|-----|-----------------|-----------|------|
| 3 | STS3215 Servo (30kg.cm, 12V) | 3 | ~19.00 | 57.00 | [AliExpress](https://de.aliexpress.com/item/1005009339011602.html) |

### Servo Control & Power

| # | Component | Qty | Unit Price (EUR) | Total (EUR) | Link |
|---|-----------|-----|-----------------|-----------|------|
| 4 | Waveshare Serial Bus Servo Driver (12V) | 2 | ~11.00 | ~22.00 | [Amazon.de](https://www.amazon.de/Waveshare-Integrates-Control-Applicable-Integrate/dp/B0CJ6TP3TP) |
| 5 | 12V Power Supply | 1 | 18.99 | 18.99 | [Amazon.de](https://www.amazon.de/Netzteil-100V-240V-Spannungswandler-Transformator-Stromversorgung/dp/B0FHNWXS27) |
| 6 | DC Splitter Cable (1-to-2, 90 deg) | 1 | 10.21 | 10.21 | [Amazon.de](https://www.amazon.de/GINTOOYUN-DC-Splitterkabel-Y-Splitter-Adapter-90-Grad-Stecker-Verlängerungskabel/dp/B09NPXTKYC) |

### Vision / Cameras

| # | Component | Qty | Unit Price (EUR) | Total (EUR) | Link | Comment |
|---|-----------|-----|-----------------|-----------|------|---------|
| 7 | Intel RealSense D435i Depth Camera | 1 | ~589.00 | ~589.00 | [Amazon.de](https://www.amazon.de/Intel-RealSense-82635D435IDK5P-RealSense-Tiefen-Kamera-D435i/dp/B07MWR2YJB) | |
| 8 | Fish-Eye Lens Camera Module (168 deg FOV, USB-C 3.0, 640x480@60Hz) | 2 | ~100.00 | ~200.00 | [AliExpress Store](https://www.aliexpress.com/store/4588015) / [Taobao](https://item.taobao.com/item.htm?id=785049105422&mi_id=0000e4KUoVlTJSdhkV74JtTjLZJkbENbeQqvv3kuw0gurcw&skuId=5875965485322) | Contact the AliExpress seller if purchasing from Taobao is not possible |

### Cables & Connectors

| # | Component | Qty | Unit Price (EUR) | Total (EUR) | Link | Comment |
|---|-----------|-----|-----------------|-----------|------|---------|
| 9 | USB-C to USB-A Cable (Anker, 1.8 m) | 2 | 3.43 | 6.86 | [Amazon.de](https://www.amazon.de/Anker-doppelt-geflochtenes-Ladekabel-Samsung-Schwarz/dp/B07DC5PPFV) | |
| 10 | JST Wire Connector Kit | 1 | ~7.50 | ~7.50 | [Amazon.de](https://www.amazon.de/Männlich-Weiblich-Adapterkabel-Verbinder-Platinenstecker/dp/B0BZHR5NCR) | For Pan-Tilt motor connections |
| 11 | 3-Pin Extension Cable | 1 | ~5.50 | ~5.50 | [Amazon.de](https://www.amazon.de/gp/product/B0F88H5DLC) | For Pan-Tilt motor connections |

### Grippers

The gripper BOM can be found in the [PincOpen repository](https://github.com/pollen-robotics/PincOpen?tab=readme-ov-file#bom-bill-of-materials).

> **Note:** The servo and control board are already included in the BOM above.

### Summary

| Category | Subtotal (EUR) |
|----------|---------------|
| Servo Motors | ~490.00 |
| Servo Control & Power | ~51.20 |
| Vision / Cameras | ~789.00 |
| Cables & Connectors | ~19.86 |
| 2x [PincOpen Gripper](https://github.com/pollen-robotics/PincOpen?tab=readme-ov-file#bom-bill-of-materials) | ~52.92 |
| **Grand Total** | **~1,402.98** |

> Prices are approximate and may vary. The mobile platform is not included; see [XLerobot docs](https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/material.html) for platform pricing.

</details>

## Setup

<details>
<summary id="link-xlerobot_pinc-into-lerobot">Link xlerobot_pinc into lerobot</summary>

Use symlinks to integrate with your `lerobot` checkout:

```bash
cd /path/to/xlerobot_pinc
./setup_lerobot_symlinks.sh /path/to/lerobot
```

This links the following into `lerobot`:
- `src/lerobot/robots/xlerobot_pinc`
- `src/lerobot/robots/so107_follower`
- `src/lerobot/model/kinematics.py`
- `src/lerobot/utils/quadratic_spline_via_ipol.py`
- `src/lerobot/scripts/motor_id_tool.py`

It does not link `examples/` or `XLeVR/`. If a target path already exists as a regular file, it is backed up with a timestamp before linking.

</details>

<details>
<summary id="usb-serial-symlinks">USB serial symlinks</summary>

Required for the default ports (`/dev/xlerobot_right_head`, `/dev/xlerobot_left_base`).

**1. Find USB ports:**

```bash
lerobot-find-port
```

See [LeRobot docs](https://huggingface.co/docs/lerobot/so101#1-find-the-usb-ports-associated-with-each-arm) for details.

**2. Identify device attributes:**

```bash
udevadm info -a -n /dev/ttyACM0
```

Look for `idVendor`, `idProduct`, and `serial`.

**3. Create udev rules:**

```bash
sudo vim /etc/udev/rules.d/99-xlerobot-serial.rules
```

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="YOUR_SERIAL_1", MODE="0777", SYMLINK+="xlerobot_right_head"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="YOUR_SERIAL_2", MODE="0777", SYMLINK+="xlerobot_left_base"
```

**4. Reload and verify:**

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/xlerobot_right_head /dev/xlerobot_left_base
```

</details>

<details>
<summary id="configure-motor-ids">Configure motor IDs</summary>

Follow the [LeRobot SO101 guide](https://huggingface.co/docs/lerobot/so101#follower) for motor ID setup.

**Single arm:**

```bash
lerobot-setup-motors \
    --robot.type=so107_follower \
    --robot.port=/dev/xlerobot_right_head
```

For the left arm, use `--robot.port=/dev/xlerobot_left_base`.

**Complete bimanual + pan-tilt setup:**

```python
from lerobot.robots.xlerobot_pinc import XLerobotPinc, XLerobotPincConfig

config = XLerobotPincConfig(
    id="ambient_xlerobot_pinc",
    has_mobile_platform=True,
    use_degrees=True
)
robot = XLerobotPinc(config)
robot.setup_motors()
```

</details>

<details>
<summary id="user-runtime-config">User runtime config</summary>

Create your local config from the template:

```bash
cp xlerobot_user_config.example.json xlerobot_user_config.json
```

Edit `xlerobot_user_config.json` with your camera paths/serials and platform settings. Leave `urdf_path` empty to use the bundled `xlerobot_pinc_urdf/robot.urdf`, or set it to a custom URDF file or directory:

```json
{
  "has_mobile_platform": false,
  "urdf_path": "",
  "cameras": {
    "left_wrist": { "index_or_path": "/dev/videoX", "fps": 60, "width": 640, "height": 480, "fourcc": "YUYV" },
    "right_wrist": { "index_or_path": "/dev/videoY", "fps": 60, "width": 640, "height": 480, "fourcc": "YUYV" },
    "head": { "serial_number_or_name": "your_realsense_serial", "fps": 60, "width": 640, "height": 480 }
  }
}
```

Load it in each shell session:

```bash
source /path/to/xlerobot_pinc/load_xlerobot_env.sh
```

</details>

## Acknowledgements

`xlerobot_pinc` is extended from [XLeRobot](https://github.com/Vector-Wangel/XLeRobot). Both the robot platform and parts of the software stack build on that upstream project, including the bundled `XLeVR` VR teleoperation module.

The gripper is based on [PincOpen](https://github.com/pollen-robotics/PincOpen) by Pollen Robotics.

See [hardware/SOURCES.md](hardware/SOURCES.md) for detailed upstream source references.
