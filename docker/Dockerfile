# ROS 2 Humble (Ubuntu 22.04 jammy) development container.
#
# Humble is supported until May 2027, so none of the EOL workarounds the old
# Foxy image needed apply here. One thing does still need handling: ROS rotated
# its apt signing key in 2025, and image layers built before then carry the
# expired one, so the key is refreshed below.
#
# Base image: `ros:humble-ros-base`, NOT `osrf/ros:humble-desktop`. The osrf
# desktop tags are published for linux/amd64 only, so they cannot run on this
# aarch64 Jetson; the official `ros` images are multi-arch (amd64 + arm64). The
# desktop tooling that image lacks is installed from apt in step 3.
FROM ros:humble-ros-base

ARG ROS_DISTRO=humble
ARG USERNAME=dogu
ARG UID=1000
ARG GID=1000
ARG WS_DIR=/home/dogu/ros2_humble_ws
# Host GIDs for GPU access, so /dev/dri is usable from inside the container.
ARG VIDEO_GID=44
ARG RENDER_GID=993
# Host GID of the 'input' group, so /dev/input/js* is readable by the
# non-root user for joy_linux_node.
ARG INPUT_GID=996
# Gazebo Classic: auto | yes | no. See step 4 -- it is amd64-only.
ARG INSTALL_SIM=auto

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# The base image and the jammy apt line below both pin this to humble. Fail
# here rather than two minutes later on an "Unable to locate package" from apt.
RUN [ "${ROS_DISTRO}" = "humble" ] || { \
      echo "ERROR: this image only builds for humble, got ROS_DISTRO='${ROS_DISTRO}'." >&2; \
      echo "A ROS_DISTRO in the build environment overrides docker/.env." >&2; \
      exit 1; \
    }

# --- 1. Refresh the ROS apt key --------------------------------------------
# Re-add the ROS repo with a current key and the architecture this image is
# actually being built for. The Foxy file hardcoded `arch=amd64`, which makes
# every ros-* install fail on aarch64.
#
# Clearing the OLD definition first is the part that has to be exact. The
# ros:humble-* images used to ship `sources.list.d/ros2.list`; they now ship
# `sources.list.d/ros2.sources` -- deb822 format, a symlink into
# /usr/share/ros-apt-source/, with the signing key inlined under `Signed-By:`.
# A glob that only sweeps `*.list` leaves that file in place, and apt then sees
# packages.ros.org declared twice with two different `Signed-By` values. That
# is a hard error, not a warning: apt aborts every subsequent command with
# "E: The list of sources could not be read." So sweep both formats.
RUN rm -f /etc/apt/sources.list.d/ros*.list /etc/apt/sources.list.d/ros*.sources \
 && apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && rm -rf /var/lib/apt/lists/*

# --- 2. Development toolchain ----------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      python3-pip python3-venv python3-dev \
      python3-colcon-common-extensions \
      python3-colcon-mixin \
      python3-rosdep python3-vcstool python3-argcomplete \
      nano vim tmux htop less tree \
      openssh-client iputils-ping net-tools iproute2 usbutils \
      libgl1-mesa-dri libgl1-mesa-glx libglu1-mesa mesa-utils \
      sudo bash-completion \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 3. ROS 2 packages -----------------------------------------------------
# ros-humble-desktop is what the ros-base image is missing: rviz2, rqt, the
# demo nodes, tf2 tooling. Everything here is published for both amd64 and
# arm64. Trim nav2/slam-toolbox if you don't need them -- they dominate the
# size now that Gazebo is separate.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop \
      ros-${ROS_DISTRO}-rqt-common-plugins \
      ros-${ROS_DISTRO}-rqt-graph \
      ros-${ROS_DISTRO}-rqt-tf-tree \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-tf2-sensor-msgs \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-joy-linux \
      ros-${ROS_DISTRO}-teleop-twist-joy \
      ros-${ROS_DISTRO}-serial-driver \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-robot-localization \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 4. Gazebo Classic (amd64 only) ----------------------------------------
# packages.ros.org has no arm64 build of ros-humble-gazebo-ros-pkgs (nor of the
# ros-gz metapackage) for jammy, so on the Jetson this step is skipped rather
# than failing the build. INSTALL_SIM=yes forces the attempt anyway.
RUN arch="$(dpkg --print-architecture)"; want="${INSTALL_SIM}"; \
    if [ "$want" = "auto" ]; then \
      if [ "$arch" = "amd64" ]; then want=yes; else want=no; fi; \
    fi; \
    if [ "$want" = "yes" ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      && apt-get clean && rm -rf /var/lib/apt/lists/*; \
    else \
      echo "Skipping Gazebo: not published for ${arch} (set INSTALL_SIM=yes to override)."; \
    fi

# --- 4b. Ignition Fortress (Gazebo Sim 6) ----------------------------------
# packages.ros.org publishes no arm64 build of ros_gz_* or ign_ros2_control,
# which is why INSTALL_SIM above skips Gazebo Classic. Ignition itself is a
# different matter: OSRF *does* publish `ignition-fortress` for arm64/jammy
# from its own repo. So the sim stack works on this Jetson after all -- install
# Fortress from OSRF here, and build the two ROS-side bridge packages
# (gazebosim/ros_gz + ros-controls/gz_ros2_control, both `humble` branch) from
# source in the workspace. See docker/README.md.
RUN install -m 0755 -d /etc/apt/keyrings \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /etc/apt/keyrings/gazebo-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends ignition-fortress \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 4c. Workspace rosdep dependencies -------------------------------------
# Exactly what `rosdep install` resolved for src/antbot plus its four external
# repos and the two Gazebo bridge repos. Baked into the image on purpose: a
# container is disposable, so anything rosdep installs at runtime disappears on
# the next `run.sh down`, and the workspace silently stops building.
# Regenerate after adding packages to src/ by re-running rosdep and diffing.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-control-cmake \
      ros-${ROS_DISTRO}-ros2-control-test-assets \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-controller-interface \
      ros-${ROS_DISTRO}-hardware-interface \
      ros-${ROS_DISTRO}-hardware-interface-testing \
      ros-${ROS_DISTRO}-realtime-tools \
      ros-${ROS_DISTRO}-control-toolbox \
      ros-${ROS_DISTRO}-generate-parameter-library \
      ros-${ROS_DISTRO}-joint-state-broadcaster \
      ros-${ROS_DISTRO}-joint-trajectory-controller \
      ros-${ROS_DISTRO}-forward-command-controller \
      ros-${ROS_DISTRO}-diff-drive-controller \
      ros-${ROS_DISTRO}-ackermann-steering-controller \
      ros-${ROS_DISTRO}-mecanum-drive-controller \
      ros-${ROS_DISTRO}-tricycle-controller \
      ros-${ROS_DISTRO}-force-torque-sensor-broadcaster \
      ros-${ROS_DISTRO}-imu-sensor-broadcaster \
      ros-${ROS_DISTRO}-rviz-imu-plugin \
      ros-${ROS_DISTRO}-camera-info-manager \
      ros-${ROS_DISTRO}-image-publisher \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-sdformat-urdf \
      ros-${ROS_DISTRO}-actuator-msgs \
      ros-${ROS_DISTRO}-gps-msgs \
      ros-${ROS_DISTRO}-nmea-msgs \
      ros-${ROS_DISTRO}-rtcm-msgs \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-backward-ros \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# NOTE: no pip upgrade of setuptools here. The Foxy image pinned 58.2.0; Humble
# expects jammy's own 59.6.0, and moving off it is what produces the
# "setup.py install is deprecated" failures in ament_python builds. Leave pip
# and setuptools at the distro versions.

# --- 5. Non-root user matching the host UID/GID ----------------------------
# Matching UID/GID is what keeps files written in the mounted workspace owned
# by you on the host instead of by root.
RUN if getent group ${GID} >/dev/null; then \
        existing=$(getent group ${GID} | cut -d: -f1); \
        [ "$existing" = "${USERNAME}" ] || groupmod -n ${USERNAME} "$existing"; \
    else \
        groupadd -g ${GID} ${USERNAME}; \
    fi \
 && if getent passwd ${UID} >/dev/null; then \
        existing=$(getent passwd ${UID} | cut -d: -f1); \
        [ "$existing" = "${USERNAME}" ] || usermod -l ${USERNAME} -d /home/${USERNAME} -m "$existing"; \
    else \
        useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME}; \
    fi \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# GPU + input device groups (created only if the host GIDs aren't taken already).
RUN getent group ${VIDEO_GID}  >/dev/null || groupadd -g ${VIDEO_GID}  hostvideo \
 && getent group ${RENDER_GID} >/dev/null || groupadd -g ${RENDER_GID} hostrender \
 && getent group ${INPUT_GID}  >/dev/null || groupadd -g ${INPUT_GID}  hostinput \
 && usermod -aG "$(getent group ${VIDEO_GID}  | cut -d: -f1)" ${USERNAME} \
 && usermod -aG "$(getent group ${RENDER_GID} | cut -d: -f1)" ${USERNAME} \
 && usermod -aG "$(getent group ${INPUT_GID}  | cut -d: -f1)" ${USERNAME}

RUN mkdir -p ${WS_DIR}/src /home/${USERNAME}/.ros \
 && chown -R ${UID}:${GID} ${WS_DIR} /home/${USERNAME}

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod 0755 /usr/local/bin/entrypoint.sh

USER ${USERNAME}
WORKDIR ${WS_DIR}

ENV ROS_DISTRO=${ROS_DISTRO} \
    WS_DIR=${WS_DIR} \
    USERNAME=${USERNAME}

# Humble is a supported distro, so plain `rosdep update` sees it -- Foxy needed
# --include-eol-distros here.
RUN rosdep update --rosdistro ${ROS_DISTRO} || true

RUN { \
      echo ''; \
      echo '# --- ROS 2 environment ---'; \
      echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
      echo "[ -f ${WS_DIR}/install/setup.bash ] && source ${WS_DIR}/install/setup.bash"; \
      echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash 2>/dev/null'; \
      echo ''; \
      echo '# --- workspace shortcuts ---'; \
      echo "alias rbuild='(cd ${WS_DIR} && colcon build --symlink-install)'"; \
      echo "alias rtest='(cd ${WS_DIR} && colcon test && colcon test-result --verbose)'"; \
      echo "alias rsource='source ${WS_DIR}/install/setup.bash'"; \
      echo "alias rclean='(cd ${WS_DIR} && rm -rf build install log)'"; \
      echo "alias rdep='(cd ${WS_DIR} && rosdep install --from-paths src --ignore-src -r -y)'"; \
    } >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
