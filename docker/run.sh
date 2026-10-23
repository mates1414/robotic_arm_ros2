#!/usr/bin/env bash
# Builds the image (if missing), starts the persistent development container and enters it.
#
#   ./docker/run.sh              interactive shell (at the workspace root)
#   ./docker/run.sh <command...> runs the command in the container with the ROS environment loaded
#   ./docker/run.sh --build      rebuilds the image
#   ./docker/run.sh --stop       stops and removes the container
#
# GUI: the X11 socket (and the Wayland socket if any) is mounted; /dev/dri and the NVIDIA GPU (if any) are passed in.
# ROS_DOMAIN_ID defaults to 21 so this host-network container does not see other ROS containers on domain 0.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "${SCRIPT_DIR}")"
IMAGE="${UR5_IMAGE:-ur5_ros2:jazzy}"
CONTAINER="${UR5_CONTAINER:-ur5_ros2_dev}"
USERNAME="$(id -un)"
CT_WS="/home/${USERNAME}/robotic_arm_ros2"

build_image() {
  docker build \
    --build-arg USERNAME="${USERNAME}" \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    -t "${IMAGE}" "${SCRIPT_DIR}"
}

case "${1:-}" in
  --build) build_image; exit 0 ;;
  --stop)  docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true; exit 0 ;;
esac

if [ -z "$(docker images -q "${IMAGE}")" ]; then
  build_image
fi

if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null || echo false)" != "true" ]; then
  docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true

  # Allow local X clients (only containers on this machine)
  if command -v xhost >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    xhost +local: >/dev/null 2>&1 || true
  fi

  ARGS=(
    --name "${CONTAINER}" -d --init
    --network host --ipc host
    -e DISPLAY="${DISPLAY:-:0}"
    -e QT_X11_NO_MITSHM=1
    -e UR5_WS="${CT_WS}"
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "${WS_DIR}:${CT_WS}"
    -w "${CT_WS}"
  )
  if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -n "${XDG_RUNTIME_DIR:-}" ]; then
    ARGS+=(-e WAYLAND_DISPLAY="${WAYLAND_DISPLAY}" -e XDG_RUNTIME_DIR=/tmp/xdg
           -v "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/tmp/xdg/${WAYLAND_DISPLAY}")
  fi
  if [ -e /dev/dri ]; then
    ARGS+=(--device /dev/dri)
  fi
  # NVIDIA: pass the GPU if nvidia-container-toolkit is installed (Gazebo rendering and cameras)
  if command -v nvidia-smi >/dev/null 2>&1 && docker info --format '{{json .Runtimes}}' | grep -q nvidia; then
    ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all)
  fi

  docker run "${ARGS[@]}" "${IMAGE}" sleep infinity >/dev/null
fi

TTY=()
[ -t 0 ] && [ -t 1 ] && TTY=(-it)

if [ $# -eq 0 ]; then
  docker exec "${TTY[@]}" "${CONTAINER}" bash
else
  docker exec "${TTY[@]}" "${CONTAINER}" bash -c "source /etc/profile.d/ros_env.sh; $*"
fi
