# syntax=docker/dockerfile:1
# Collider dev image: ArduPilot's ROS 2 build image + Gazebo Harmonic + sim tools.
# Holds only the toolchain; the workspace (src/, build/, install/) is bind-mounted at /ws.
# Build context is the workspace root, see docker-compose.yml.

# ardupilot/ardupilot-dev-ros:latest (ROS 2 humble), pinned 2026-10-25
ARG BASE_IMAGE=ardupilot/ardupilot-dev-ros@sha256:95f3249a8091318192e86b42cb43abde555ae2fdc3f914e91ebe42d67c783bd2
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble
ENV GZ_VERSION=harmonic

# Gazebo Harmonic: apt repo, rosdep keys, simulator
RUN wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && echo "yaml https://github.com/osrf/osrf-rosdep/raw/master/gz/gazebo.yaml" \
        > /etc/ros/rosdep/sources.list.d/00-gazebo.list \
    && apt-get update && apt-get install --no-install-recommends -y gz-harmonic \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS / sim tools not declared by any package.xml
RUN apt-get update && apt-get install --no-install-recommends -y \
    gstreamer1.0-gl \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    python3-geopy \
    python3-matplotlib \
    python3-opencv \
    python3-scipy \
    python3-wxgtk4.0 \
    ros-"${ROS_DISTRO}"-image-transport-plugins \
    ros-"${ROS_DISTRO}"-rqt-image-view \
    ros-"${ROS_DISTRO}"-rqt-plot \
    ros-"${ROS_DISTRO}"-rqt-topic \
    ros-"${ROS_DISTRO}"-rviz2 \
    ros-"${ROS_DISTRO}"-tf-transformations \
    ros-"${ROS_DISTRO}"-topic-tools \
    ros-"${ROS_DISTRO}"-vision-opencv \
    ros-"${ROS_DISTRO}"-web-video-server \
    ros-"${ROS_DISTRO}"-xacro \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --no-cache-dir MAVProxy==1.8.74

# Dev tools + SSH access (the container is used over an SSH tunnel)
RUN apt-get update && apt-get install --no-install-recommends -y \
    htop \
    less \
    lsof \
    openssh-server \
    screen \
    vim \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN cat > /etc/ssh/sshd_config.d/collider.conf <<'EOF'
PasswordAuthentication no
PermitRootLogin no
AllowAgentForwarding yes
X11Forwarding yes
HostKey /etc/ssh/host_keys/ssh_host_ed25519_key
HostKey /etc/ssh/host_keys/ssh_host_rsa_key
EOF

# Gazebo renders on the GPU through EGL; libEGL_nvidia is injected by the NVIDIA runtime
RUN mkdir -p /usr/share/glvnd/egl_vendor.d \
    && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}\n' \
        > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Non-root user matching the host UID, so files in the mounted workspace stay yours
ARG USERNAME=dev
ARG HOST_UID=1000
ARG HOST_GID=1000
RUN groupadd -g "${HOST_GID}" "${USERNAME}" \
    && useradd -m -u "${HOST_UID}" -g "${HOST_GID}" -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/"${USERNAME}" \
    && install -d -m 700 -o "${USERNAME}" -g "${USERNAME}" /home/"${USERNAME}"/.ssh \
    && install -d -o "${USERNAME}" -g "${USERNAME}" /ccache /ws
ENV CCACHE_DIR=/ccache

# Shell setup for every interactive shell (in /etc so a persistent home volume doesn't hide updates)
RUN cat >> /etc/bash.bashrc <<'EOF'

# --- collider ---
# sshd starts sessions with a clean env; the entrypoint saves the container env here
[ -f /etc/container.env ] && source /etc/container.env
source /opt/ros/"${ROS_DISTRO}"/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/src/ardupilot/Tools/completion/completion.bash ] && source /ws/src/ardupilot/Tools/completion/completion.bash
export PATH=/ws/src/ardupilot/Tools/autotest:$PATH
cd /ws
EOF

# Dependencies declared by the workspace. Only package.xml files are in the build context
# (Dockerfile.dockerignore), so this layer rebuilds only when dependencies change.
COPY src /tmp/deps/src
RUN apt-get update && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/deps/src /tmp/deps/src/collider/interfaces/collider_interfaces \
        --ignore-src -r -y --rosdistro "${ROS_DISTRO}" \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/deps

# YOLO for car_detector_node. Ultralytics brings torch with CUDA (several GB).
# numpy 1.24: ROS Humble needs numpy 1.x, and apt's scipy 1.8 supports only <1.25.
# setuptools pinned: torch would upgrade it, and newer ones break colcon --symlink-install.
# numpy, sympy and mpmath also come from apt, which pip cannot uninstall; --ignore-installed
# puts newer copies in /usr/local (first on sys.path) and leaves the apt ones alone.
RUN python -m pip install --no-cache-dir --ignore-installed \
        numpy==1.24.4 sympy==1.14.0 mpmath==1.3.0 \
    && python -m pip install --no-cache-dir numpy==1.24.4 setuptools==65.5.1 ultralytics==8.4.192
# VisDrone-tuned weights baked in, so nothing is downloaded at runtime; the checksum makes
# the build fail if the file ever changes upstream. ADD --chmod would also apply to the new
# directory (no x bit, unreadable for dev), so permissions are set afterwards.
ADD --checksum=sha256:97d78d933f46d693f68817fca253c728444aafcec92be42c86ad703ed67d40d5 \
    https://huggingface.co/dronefreak/visdrone-yolo11n/resolve/7207f1d976e1e88a3806e8aadc438424d6375237/best.pt \
    /opt/models/yolo11n-visdrone.pt
RUN chmod 755 /opt/models && chmod 644 /opt/models/yolo11n-visdrone.pt

COPY src/collider/docker/entrypoint.sh /usr/local/bin/entrypoint.sh
EXPOSE 22
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
