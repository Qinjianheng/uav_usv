#!/usr/bin/env bash
set -eo pipefail

WS_ROOT="${UAV_USV_WS:-/home/qin/data/uav_usv}"
PX4_ROOT="${PX4_ROOT:-/home/qin/Projects/PX4-Autopilot}"
BUILD_WORKSPACE=true

if [[ "${1:-}" == "--no-build" ]]; then
    BUILD_WORKSPACE=false
elif [[ $# -gt 0 ]]; then
    echo "Usage: $0 [--no-build]" >&2
    exit 2
fi

if [[ ! -d "${WS_ROOT}/src" ]]; then
    echo "Workspace not found: ${WS_ROOT}" >&2
    exit 1
fi

if [[ ! -d "${PX4_ROOT}" ]]; then
    echo "PX4 source directory not found: ${PX4_ROOT}" >&2
    exit 1
fi

if ! pgrep -f '[Q]GroundControl' >/dev/null; then
    echo "QGroundControl is not running."
    echo "Open QGroundControl first, then run this script again."
    exit 1
fi

source /opt/ros/humble/setup.bash

if ${BUILD_WORKSPACE}; then
    "${WS_ROOT}/scripts/build_workspace.sh"
elif [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
    echo "Workspace has not been built; omit --no-build for the first run." >&2
    exit 1
fi

source "${WS_ROOT}/install/setup.bash"
export UAV_USV_WS="${WS_ROOT}"

echo "Starting PX4 SITL and Gazebo..."
gnome-terminal --title="PX4 SITL" -- bash -lc "
cd '${PX4_ROOT}' &&
make px4_sitl gz_x500;
exec bash"

echo "Waiting 15 seconds for PX4 initialization..."
sleep 15

echo "Starting Micro XRCE-DDS Agent..."
gnome-terminal --title="Micro XRCE-DDS Agent" -- bash -lc "
MicroXRCEAgent udp4 -p 8888;
exec bash"

echo "Waiting 5 seconds for DDS connection..."
sleep 5

echo "Starting UAV-USV baseline experiment..."
gnome-terminal --title="UAV-USV experiment" -- bash -lc "
export UAV_USV_WS='${WS_ROOT}' &&
source /opt/ros/humble/setup.bash &&
source '${WS_ROOT}/install/setup.bash' &&
cd '${WS_ROOT}' &&
ros2 launch uav_usv_bringup baseline_intercept.launch.py;
exec bash"

echo "Waiting 5 seconds for ROS 2 nodes..."
sleep 5

publish_command()
{
    ros2 topic pub --once \
        /simulation/impact/command \
        std_msgs/msg/String \
        "{data: '$1'}"
}

echo
echo "Two-stage control is ready."
echo "  X: take off and enter FOLLOW MODE"
echo "  Y: start interception after FOLLOW MODE"
echo "  Q: leave this command console"

while true; do
    read -r -p "Command [X/Y/Q]: " command
    command=${command^^}

    case "${command}" in
        X)
            publish_command X
            ;;
        Y)
            publish_command Y
            ;;
        Q)
            echo "Command console closed; experiment terminals remain open."
            break
            ;;
        *)
            echo "Please enter X, Y, or Q."
            ;;
    esac
done
