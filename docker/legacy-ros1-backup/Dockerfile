ARG ROS_VERSION="foxy"
FROM osrf/ros:foxy-desktop-full

ARG CATKIN_WORKSPACE_DIR="/home/dogux/ros2_foxy_ws"
WORKDIR ${CATKIN_WORKSPACE_DIR}
RUN echo "------------======running docker=====------"
ARG ROS_VERSION
ARG USERNAME=rgt
ENV USERNAME=${USERNAME}
RUN apt-get update && apt-get install -y --no-install-recommends \
  # Catkin and ROS tools
  # && add-apt-repository ppa:kisak/kisak-mesa -y \
  # && apt-get update && apt-get install -y \
  ros-${ROS_VERSION}-catkin \
  python-catkin-tools \
  python3-vcstool \
  python-rospkg \
  ros-${ROS_VERSION}-serial \
  ros-${ROS_VERSION}-tf2-sensor-msgs \
  ros-${ROS_VERSION}-move-base-msgs \
  libgl1-mesa-dri \
  libgl1-mesa-glx \
  mesa-utils \
  ros-foxy-rviz \
  ros-foxy-gazebo-ros \
  ros-foxy-amcl \
  ros-foxy-map-server \
  # ros navigation
  ros-foxy-mbf-costmap-core \
  ros-foxy-mbf-msgs \
  ros-foxy-libg2o \
  

  \
  # Basic development environment
  build-essential \
  git \
  python-pip \
  python3-pip \
  python3-venv \
  python3-tk \
  nano \
  vim \
  tmux \
  htop \
  openssh-client \
  \


  # Install Stage and stage_ros
  ros-${ROS_VERSION}-stage \
  ros-${ROS_VERSION}-stage-ros \
  # navigation
  libsdl1.2-dev \
  # Clean up
  && pip3 install PyYAML catkin_pkg rospkg \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Set the user environment variable
ENV USER=rgt
RUN useradd -ms /bin/bash ${USERNAME}

RUN python3 -m venv /opt/venv \
  && /opt/venv/bin/pip install --upgrade pip setuptools wheel \
  && chown -R ${USERNAME}:${USERNAME} /opt/venv

ENV ROS_VERSION=${ROS_VERSION}

ARG UID=1000
ARG GID=1000
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME}

# Set permissions for catkin workspace and ROS directory
RUN mkdir -p /home/${USERNAME}/.ros \
  && chown -R ${UID}:${GID} /home/${USERNAME} \
  && chmod -R u+rwX /home/${USERNAME}/.ros

RUN chown -R rgt:rgt /home/${USERNAME}/catkin_ws

# Create the /tmp/runtime-rgt directory with correct permissions and ownership
RUN mkdir -p /tmp/runtime-rgt && \
    chmod 0700 /tmp/runtime-rgt && \
    chown rgt:rgt /tmp/runtime-rgt 
    

USER ${USERNAME}
ENV PATH="/opt/venv/bin:$PATH"

# Initialize and configure catkin workspace
RUN catkin config --extend /opt/ros/${ROS_VERSION} && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "alias rsource='source ${CATKIN_WORKSPACE_DIR}/devel/setup.bash'" >> /home/${USERNAME}/.bashrc \
  && echo "alias rbuild='(cd ${CATKIN_WORKSPACE_DIR} && catkin build)'" >> /home/${USERNAME}/.bashrc \
  && echo "alias rclean='(cd ${CATKIN_WORKSPACE_DIR} && catkin clean -y)'" >> /home/${USERNAME}/.bashrc \
  && echo "[ -f ${CATKIN_WORKSPACE_DIR}/devel/setup.bash ] && source ${CATKIN_WORKSPACE_DIR}/devel/setup.bash || source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
  && echo "source /opt/venv/bin/activate" >> /home/${USERNAME}/.bashrc
