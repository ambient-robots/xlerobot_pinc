#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "This script must be sourced to export environment variables."
    echo "Usage: source ${0} [path/to/xlerobot_user_config.json]"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${1:-${SCRIPT_DIR}/xlerobot_user_config.json}"

if [[ ! -f "${CONFIG_PATH}" ]]; then
    echo "Config file not found: ${CONFIG_PATH}"
    echo "Create it from: ${SCRIPT_DIR}/xlerobot_user_config.example.json"
    return 1
fi

if ! EXPORT_LINES="$(
python - "${CONFIG_PATH}" <<'PY'
import json
import shlex
import sys
from pathlib import Path

config_path = Path(sys.argv[1])
with config_path.open("r", encoding="utf-8") as f:
    cfg = json.load(f)

if "has_mobile_platform" not in cfg:
    raise SystemExit("Missing required key: has_mobile_platform")

has_mobile_platform = cfg["has_mobile_platform"]
if not isinstance(has_mobile_platform, bool):
    raise SystemExit("has_mobile_platform must be a JSON boolean (true/false)")

if "urdf_path" not in cfg:
    raise SystemExit("Missing required key: urdf_path")

urdf_path = cfg["urdf_path"]
if not isinstance(urdf_path, str) or not urdf_path.strip():
    raise SystemExit("urdf_path must be a non-empty JSON string")

print(f"export XLEROBOT_HAS_MOBILE_PLATFORM={'1' if has_mobile_platform else '0'}")
print(f"export XLEROBOT_URDF_PATH={shlex.quote(urdf_path)}")

if "cameras" not in cfg:
    raise SystemExit("Missing required key: cameras")

cameras_cfg = cfg["cameras"]
if not isinstance(cameras_cfg, dict):
    raise SystemExit("cameras must be a JSON object")


def get_camera_cfg(camera_name: str) -> dict:
    if camera_name not in cameras_cfg:
        raise SystemExit(f"Missing required key: cameras.{camera_name}")
    camera_cfg = cameras_cfg[camera_name]
    if not isinstance(camera_cfg, dict):
        raise SystemExit(f"cameras.{camera_name} must be a JSON object")
    return camera_cfg


camera_cfgs = {
    "left_wrist": get_camera_cfg("left_wrist"),
    "right_wrist": get_camera_cfg("right_wrist"),
    "head": get_camera_cfg("head"),
}


def get_required_nonempty_str(camera_name: str, field_name: str) -> str:
    value = camera_cfgs[camera_name].get(field_name)
    if not isinstance(value, str) or not value.strip():
        raise SystemExit(f"cameras.{camera_name}.{field_name} must be a non-empty JSON string")
    return value.strip()


def get_optional_positive_int(camera_name: str, field_name: str):
    value = camera_cfgs[camera_name].get(field_name)
    if value is None:
        return None
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise SystemExit(f"cameras.{camera_name}.{field_name} must be a positive JSON integer")
    return value


def get_optional_fourcc(camera_name: str, field_name: str):
    value = camera_cfgs[camera_name].get(field_name)
    if value is None:
        return None
    if not isinstance(value, str) or len(value.strip()) != 4:
        raise SystemExit(f"cameras.{camera_name}.{field_name} must be a 4-character JSON string")
    return value.strip()


required_string_fields = [
    ("left_wrist", "index_or_path", "XLEROBOT_LEFT_WRIST_INDEX_OR_PATH"),
    ("right_wrist", "index_or_path", "XLEROBOT_RIGHT_WRIST_INDEX_OR_PATH"),
    ("head", "serial_number_or_name", "XLEROBOT_HEAD_SERIAL_NUMBER_OR_NAME"),
]

for camera_name, field_name, env_key in required_string_fields:
    print(f"export {env_key}={shlex.quote(get_required_nonempty_str(camera_name, field_name))}")

optional_int_fields = [
    ("left_wrist", "fps", "XLEROBOT_LEFT_WRIST_FPS"),
    ("left_wrist", "width", "XLEROBOT_LEFT_WRIST_WIDTH"),
    ("left_wrist", "height", "XLEROBOT_LEFT_WRIST_HEIGHT"),
    ("right_wrist", "fps", "XLEROBOT_RIGHT_WRIST_FPS"),
    ("right_wrist", "width", "XLEROBOT_RIGHT_WRIST_WIDTH"),
    ("right_wrist", "height", "XLEROBOT_RIGHT_WRIST_HEIGHT"),
    ("head", "fps", "XLEROBOT_HEAD_FPS"),
    ("head", "width", "XLEROBOT_HEAD_WIDTH"),
    ("head", "height", "XLEROBOT_HEAD_HEIGHT"),
]

for camera_name, field_name, env_key in optional_int_fields:
    value = get_optional_positive_int(camera_name, field_name)
    if value is None:
        print(f"unset {env_key}")
    else:
        print(f"export {env_key}={value}")

optional_fourcc_fields = [
    ("left_wrist", "fourcc", "XLEROBOT_LEFT_WRIST_FOURCC"),
    ("right_wrist", "fourcc", "XLEROBOT_RIGHT_WRIST_FOURCC"),
]

for camera_name, field_name, env_key in optional_fourcc_fields:
    value = get_optional_fourcc(camera_name, field_name)
    if value is None:
        print(f"unset {env_key}")
    else:
        print(f"export {env_key}={shlex.quote(value)}")
PY
)"
then
    return 1
fi

eval "${EXPORT_LINES}"
echo "Loaded XLerobot env from ${CONFIG_PATH}"
echo "  XLEROBOT_HAS_MOBILE_PLATFORM=${XLEROBOT_HAS_MOBILE_PLATFORM}"
echo "  XLEROBOT_URDF_PATH=${XLEROBOT_URDF_PATH}"
echo "  left_wrist: index_or_path=${XLEROBOT_LEFT_WRIST_INDEX_OR_PATH}, fps=${XLEROBOT_LEFT_WRIST_FPS:-<default 60>}, width=${XLEROBOT_LEFT_WRIST_WIDTH:-<default 640>}, height=${XLEROBOT_LEFT_WRIST_HEIGHT:-<default 480>}, fourcc=${XLEROBOT_LEFT_WRIST_FOURCC:-<default YUYV>}"
echo "  right_wrist: index_or_path=${XLEROBOT_RIGHT_WRIST_INDEX_OR_PATH}, fps=${XLEROBOT_RIGHT_WRIST_FPS:-<default 60>}, width=${XLEROBOT_RIGHT_WRIST_WIDTH:-<default 640>}, height=${XLEROBOT_RIGHT_WRIST_HEIGHT:-<default 480>}, fourcc=${XLEROBOT_RIGHT_WRIST_FOURCC:-<default YUYV>}"
echo "  head: serial_number_or_name=${XLEROBOT_HEAD_SERIAL_NUMBER_OR_NAME}, fps=${XLEROBOT_HEAD_FPS:-<default 60>}, width=${XLEROBOT_HEAD_WIDTH:-<default 640>}, height=${XLEROBOT_HEAD_HEIGHT:-<default 480>}"
