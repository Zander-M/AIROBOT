FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/opt/ros_ws
ENV PACKAGE_NAME=ros2_aruco_position
ENV PACKAGE_MOUNT=/workspace/ros2_aruco_position
ENV RUNTIME_USER=${USERNAME}
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-${USERNAME}

RUN apt-get update && apt-get install -y --no-install-recommends \
    gosu \
    tmux \
    python3-rosdep \
    python3-opencv \
    ros-humble-tf2-ros \
    sudo \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -f video && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} && \
    usermod -aG video ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p ${ROS_WS}/src ${PACKAGE_MOUNT} && \
    mkdir -p ${XDG_RUNTIME_DIR} && \
    chown -R ${USERNAME}:${USERNAME} ${ROS_WS} /workspace ${XDG_RUNTIME_DIR} && \
    chmod 700 ${XDG_RUNTIME_DIR}

RUN rosdep update || true

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f ${ROS_WS}/install/setup.bash ]; then source ${ROS_WS}/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR ${ROS_WS}
VOLUME ["/workspace/ros2_aruco_position"]

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ros2_aruco_position", "aruco_tf.launch.py"]
