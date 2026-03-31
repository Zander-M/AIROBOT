FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV COLCON_WS=/home/ros/ros_ws

# -------------------------
# System packages
# -------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    tmux \
    vim \
    fzf \
    less \
    curl \
    git \
    bash-completion \
    ca-certificates \
    wget \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p /home/${USERNAME}/ros_ws/src && \
    chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# -------------------------
# tmux config
# -------------------------
RUN printf '%s\n' \
    'set -g mouse on' \
    'set -g history-limit 100000' \
    'set -g default-terminal "screen-256color"' \
    'setw -g mode-keys vi' \
    'bind r source-file ~/.tmux.conf \; display-message "tmux.conf reloaded"' \
    > /home/ros/.tmux.conf

# -------------------------
# bash config
# -------------------------
RUN printf '%s\n' \
    '# ROS setup' \
    'source /opt/ros/${ROS_DISTRO}/setup.bash' \
    '' \
    '# Workspace setup (if exists)' \
    'if [ -f ${COLCON_WS}/install/setup.bash ]; then source ${COLCON_WS}/install/setup.bash; fi' \
    '' \
    '# Bash completion' \
    '[ -f /usr/share/bash-completion/bash_completion ] && source /usr/share/bash-completion/bash_completion' \
    '' \
    '# fzf keybindings (Ctrl+R etc.)' \
    '[ -f /usr/share/doc/fzf/examples/key-bindings.bash ] && source /usr/share/doc/fzf/examples/key-bindings.bash' \
    '[ -f /usr/share/doc/fzf/examples/completion.bash ] && source /usr/share/doc/fzf/examples/completion.bash' \
    '' \
    '# Misc' \
    'export TERM=xterm-256color' \
    'alias ll="ls -alF"' \
    'alias la="ls -A"' \
    'alias l="ls -CF"' \
    >> /home/ros/.bashrc

WORKDIR ${COLCON_WS}

CMD ["/bin/bash"]