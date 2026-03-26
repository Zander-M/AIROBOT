#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

mkdir -p "${ROS_WS}/src"

if [ ! -f "${PACKAGE_MOUNT}/package.xml" ]; then
  echo "Expected mounted package at ${PACKAGE_MOUNT}, but package.xml was not found."
  echo "Run the container with: -v \$(pwd):${PACKAGE_MOUNT}"
  exit 1
fi

ln -sfn "${PACKAGE_MOUNT}" "${ROS_WS}/src/${PACKAGE_NAME}"

if [ "$(id -u)" = "0" ]; then
  for video_device in /dev/video*; do
    if [ ! -c "${video_device}" ]; then
      continue
    fi

    video_gid="$(stat -c '%g' "${video_device}")"
    if ! getent group "${video_gid}" > /dev/null; then
      groupadd --gid "${video_gid}" "hostvideo-${video_gid}"
    fi

    video_group="$(getent group "${video_gid}" | cut -d: -f1)"
    usermod -aG "${video_group}" "${RUNTIME_USER}"
  done

  apt-get update
  rosdep install --from-paths "${ROS_WS}/src" --ignore-src -r -y
  chown -R "${RUNTIME_USER}:${RUNTIME_USER}" "${ROS_WS}"
  exec gosu "${RUNTIME_USER}" /entrypoint.sh "$@"
fi

cd "${ROS_WS}"
colcon build --packages-select "${PACKAGE_NAME}"

source "${ROS_WS}/install/setup.bash"
exec "$@"
