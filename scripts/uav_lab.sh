#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
    pwd -P
)"

WS_ROOT="$(
    cd -- "${SCRIPT_DIR}/.."
    pwd -P
)"

export UAV_USV_WS="${WS_ROOT}"

echo "=================================================="
echo "UAV-USV one-click launcher"
echo "Workspace:"
echo "  ${UAV_USV_WS}"

if git -C "${UAV_USV_WS}" rev-parse \
    --is-inside-work-tree >/dev/null 2>&1; then
    echo "Branch:"
    echo "  $(git -C "${UAV_USV_WS}" branch --show-current)"
    echo "Commit:"
    echo "  $(git -C "${UAV_USV_WS}" rev-parse --short HEAD)"
fi

echo "=================================================="

exec "${WS_ROOT}/scripts/start_px4_ros2.sh" "$@"
