#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091
set -eo pipefail

WS_ROOT="${UAV_USV_WS:-/home/qin/data/uav_usv}"
PX4_ROOT="${PX4_ROOT:-/home/qin/Projects/PX4-Autopilot}"
OCEAN_WORLD="${WS_ROOT}/src/uav_usv_bringup/worlds/ocean.sdf"
PX4_GZ_ENV="${PX4_ROOT}/build/px4_sitl_default/rootfs/gz_env.sh"
CUSTOM_GZ_MODELS="${WS_ROOT}/src/uav_usv_bringup/models"
QGC_APPIMAGE="${QGC_APPIMAGE:-/home/qin/桌面/QGroundControl-x86_64.AppImage}"
BUILD_WORKSPACE=true
CAMERA_STARTUP_TIMEOUT="${CAMERA_STARTUP_TIMEOUT:-180}"

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

if [[ ! -f "${OCEAN_WORLD}" ]]; then
    echo "Ocean world not found: ${OCEAN_WORLD}" >&2
    exit 1
fi

if ! [[ "${CAMERA_STARTUP_TIMEOUT}" =~ ^[1-9][0-9]*$ ]]; then
    echo "CAMERA_STARTUP_TIMEOUT must be a positive integer." >&2
    exit 2
fi

for required_command in gnome-terminal gz MicroXRCEAgent timeout; do
    if ! command -v "${required_command}" >/dev/null; then
        echo "Required command not found: ${required_command}" >&2
        exit 1
    fi
done

if ! pgrep -f '[Q]GroundControl' >/dev/null; then
    if [[ ! -x "${QGC_APPIMAGE}" ]]; then
        echo "QGroundControl is not running and its AppImage was not found:" >&2
        echo "  ${QGC_APPIMAGE}" >&2
        echo "Set QGC_APPIMAGE to the executable path and retry." >&2
        exit 1
    fi

    echo "Starting QGroundControl..."
    qgc_log="${TMPDIR:-/tmp}/uav_usv_qgroundcontrol.log"
    nohup "${QGC_APPIMAGE}" >"${qgc_log}" 2>&1 &

    qgc_ready=false
    for _ in {1..15}; do
        if pgrep -f '[Q]GroundControl' >/dev/null; then
            qgc_ready=true
            break
        fi
        sleep 1
    done

    if ! ${qgc_ready}; then
        echo "QGroundControl did not start. See ${qgc_log}" >&2
        exit 1
    fi
fi

if timeout 3s gz topic -e -t /world/default/clock -n 1 \
    >/dev/null 2>&1; then
    echo "A Gazebo world is already running." >&2
    echo "Close the existing Gazebo/PX4 session, then retry." >&2
    exit 1
fi

# Gazebo Transport can retain a stopped world's topic discovery records for a
# short time.  If no live clock is present, a PX4 process left by an aborted
# launch is orphaned and must not attach to the next Gazebo instance.
PX4_EXECUTABLE="$(readlink -f \
    "${PX4_ROOT}/build/px4_sitl_default/bin/px4" 2>/dev/null || true)"
if [[ -n "${PX4_EXECUTABLE}" ]]; then
    mapfile -t stale_px4_pids < <(
        pgrep -f "^${PX4_EXECUTABLE}([[:space:]]|$)" || true
    )
    if [[ ${#stale_px4_pids[@]} -gt 0 ]]; then
        echo "Stopping orphaned PX4 SITL from an earlier launch..."
        kill -INT "${stale_px4_pids[@]}" 2>/dev/null || true
        for _ in {1..10}; do
            px4_still_running=false
            for stale_pid in "${stale_px4_pids[@]}"; do
                if kill -0 "${stale_pid}" 2>/dev/null; then
                    px4_still_running=true
                    break
                fi
            done
            if ! ${px4_still_running}; then
                break
            fi
            sleep 0.2
        done
        if ${px4_still_running}; then
            echo "Orphaned PX4 SITL did not stop cleanly." >&2
            echo "Close its PX4 SITL terminal, then retry." >&2
            exit 1
        fi
    fi
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

if [[ ! -f "${PX4_GZ_ENV}" ]]; then
    echo "Preparing PX4 SITL build and Gazebo environment..."
    (
        cd "${PX4_ROOT}"
        make px4_sitl_default
    )
fi

if [[ ! -f "${PX4_GZ_ENV}" ]]; then
    echo "PX4 Gazebo environment not found: ${PX4_GZ_ENV}" >&2
    exit 1
fi

source "${PX4_GZ_ENV}"
export GZ_SIM_RESOURCE_PATH="${CUSTOM_GZ_MODELS}:${GZ_SIM_RESOURCE_PATH:-}"
export PX4_GZ_MODELS="${CUSTOM_GZ_MODELS}"

echo "Starting Gazebo ocean world..."
gnome-terminal --title="Gazebo Ocean" -- bash -lc "
source '${PX4_GZ_ENV}' &&
export GZ_SIM_RESOURCE_PATH='${CUSTOM_GZ_MODELS}':\${GZ_SIM_RESOURCE_PATH:-} &&
gz sim -r '${OCEAN_WORLD}';
exec bash"

echo "Waiting for Gazebo ocean world..."
gazebo_ready=false
for _ in {1..30}; do
    gazebo_services="$(gz service -l 2>/dev/null || true)"
    if grep -Fxq '/world/default/create' <<< "${gazebo_services}"; then
        gazebo_ready=true
        break
    fi
    sleep 1
done

if ! ${gazebo_ready}; then
    echo "Gazebo ocean world did not become ready within 30 seconds." >&2
    exit 1
fi

echo "Starting PX4 SITL in standalone Gazebo mode..."
gnome-terminal --title="PX4 SITL" -- bash -lc "
source '${PX4_GZ_ENV}' &&
export GZ_SIM_RESOURCE_PATH='${CUSTOM_GZ_MODELS}':\${GZ_SIM_RESOURCE_PATH:-} &&
export PX4_GZ_MODELS='${CUSTOM_GZ_MODELS}' &&
export PX4_GZ_STANDALONE=1 &&
export PX4_GZ_WORLD=default &&
# Face Gazebo +Y (local NED north), where the USV starts 20 m away.
export PX4_GZ_MODEL_POSE='0,0,0,0,0,1.57079632679' &&
cd '${PX4_ROOT}' &&
make px4_sitl gz_x500_mono_cam;
exec bash"

echo "Waiting up to ${CAMERA_STARTUP_TIMEOUT} seconds for PX4 and front ToF..."
camera_topics_ready=false
gazebo_topics=""
for ((elapsed = 0; elapsed < CAMERA_STARTUP_TIMEOUT; elapsed++)); do
    gazebo_topics="$(timeout 3s gz topic -l 2>/dev/null || true)"
    if grep -Fxq '/uav/camera/front/image' <<< "${gazebo_topics}" \
        && grep -Fxq '/uav/camera/front/depth_image' \
            <<< "${gazebo_topics}"; then
        camera_topics_ready=true
        break
    fi
    if ((elapsed > 0 && elapsed % 15 == 0)); then
        echo "Still waiting for PX4/front ToF (${elapsed}s)..."
    fi
    sleep 1
done

if ! ${camera_topics_ready}; then
    echo "Front ToF topics were not available within" \
        "${CAMERA_STARTUP_TIMEOUT} seconds." >&2
    echo "Expected /uav/camera/front/image and" >&2
    echo "  /uav/camera/front/depth_image." >&2
    echo "Available camera-related topics:" >&2
    rg -i 'camera|image|depth' <<< "${gazebo_topics}" >&2 || true
    echo "Check the PX4 SITL terminal for model-spawn errors." >&2
    exit 1
fi

echo "Front ToF camera is publishing aligned RGB and depth images."

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
