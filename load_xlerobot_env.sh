#!/usr/bin/env bash
set -euo pipefail

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

EXPORT_LINES="$(
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
PY
)"

eval "${EXPORT_LINES}"
echo "Loaded XLerobot env from ${CONFIG_PATH}"
echo "  XLEROBOT_HAS_MOBILE_PLATFORM=${XLEROBOT_HAS_MOBILE_PLATFORM}"
echo "  XLEROBOT_URDF_PATH=${XLEROBOT_URDF_PATH}"
