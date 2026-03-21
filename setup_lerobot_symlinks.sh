#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./setup_lerobot_symlinks.sh [path/to/lerobot] [path/to/xlerobot_pinc]
#
# Defaults:
#   lerobot:      ../lerobot (relative to xlerobot_pinc repo root)
#   xlerobot_pinc: inferred from this script location

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XLEROBOT_REPO_DEFAULT="${SCRIPT_DIR}"
LEROBOT_REPO_DEFAULT="$(cd "${SCRIPT_DIR}/../lerobot" 2>/dev/null && pwd || true)"

LEROBOT_REPO="${1:-${LEROBOT_REPO_DEFAULT}}"
XLEROBOT_REPO="${2:-${XLEROBOT_REPO_DEFAULT}}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

if [[ -z "${LEROBOT_REPO}" || ! -d "${LEROBOT_REPO}" ]]; then
    echo "Error: lerobot directory not found: ${LEROBOT_REPO}"
    echo "Usage: ./setup_lerobot_symlinks.sh /path/to/lerobot [/path/to/xlerobot_pinc]"
    exit 1
fi

if [[ ! -d "${XLEROBOT_REPO}" ]]; then
    echo "Error: xlerobot_pinc directory not found: ${XLEROBOT_REPO}"
    exit 1
fi

link_path() {
    local src_rel="$1"
    local dst_rel="$2"
    local src_abs="${XLEROBOT_REPO}/${src_rel}"
    local dst_abs="${LEROBOT_REPO}/${dst_rel}"

    if [[ ! -e "${src_abs}" ]]; then
        echo "Skip missing source: ${src_abs}"
        return 0
    fi

    mkdir -p "$(dirname "${dst_abs}")"

    if [[ -L "${dst_abs}" ]]; then
        local current_target
        current_target="$(readlink "${dst_abs}")"
        if [[ "${current_target}" == "${src_abs}" ]]; then
            echo "Already linked: ${dst_rel}"
            return 0
        fi
    elif [[ -e "${dst_abs}" ]]; then
        local backup="${dst_abs}.bak.${TIMESTAMP}"
        mv "${dst_abs}" "${backup}"
        echo "Backed up existing path: ${dst_abs} -> ${backup}"
    fi

    ln -sfn "${src_abs}" "${dst_abs}"
    echo "Linked: ${dst_rel} -> ${src_abs}"
}

# Core robot integrations
link_path "src/lerobot/robots/xlerobot_pinc" "src/lerobot/robots/xlerobot_pinc"
link_path "src/lerobot/robots/so107_follower" "src/lerobot/robots/so107_follower"
link_path "src/lerobot/model/kinematics.py" "src/lerobot/model/kinematics.py"
link_path "src/lerobot/utils/quadratic_spline_via_ipol.py" "src/lerobot/utils/quadratic_spline_via_ipol.py"
link_path "src/lerobot/scripts/motor_id_tool.py" "src/lerobot/scripts/motor_id_tool.py"

echo
echo "Done."
echo "lerobot repo: ${LEROBOT_REPO}"
echo "xlerobot_pinc repo: ${XLEROBOT_REPO}"
