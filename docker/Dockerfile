#########################################
##  Base Image for Motion Planning  ##
#########################################

FROM osrf/ros:jazzy-desktop
ENV ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Basic setup
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends --allow-unauthenticated \
    autoconf \
    automake \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    g++ \
    git \
    iproute2 \
    iputils-ping \
    libxext-dev \
    libx11-dev \
    make \
    mc \
    mesa-utils \
    nano \
    vim \
    tree \
    pkg-config \
    software-properties-common \
    sudo \
    tmux \
    tzdata \
    xclip \
    x11proto-gl-dev \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-pip \
    screen \
    figlet \
    toilet \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# install VS Code server
RUN curl -fsSL https://code-server.dev/install.sh | sh

# install python extension
RUN code-server --install-extension ms-python.python
COPY extensions/ms-iot.vscode-ros-0.9.6.vsix .
RUN code-server --install-extension ms-iot.vscode-ros-0.9.6.vsix

RUN pip install ipython roboticstoolbox-python==1.1.1 python-fcl==0.7.0.8 transforms3d==0.4.2 --break-system-packages && \
    export PATH=/home/robot/.local/bin:$PATH

# Set datetime and timezone correctly
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' | tee -a /etc/timezone

ENV DEBIAN_FRONTEND=dialog

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-moveit* \
    ros-jazzy-ur-*

RUN apt-get update \
    && apt-get install -y ros-jazzy-gz-*

# Add OSRF Gazebo repository and key, then install Gazebo development library
# needed for gz completions
RUN apt-get update && apt-get install -y --no-install-recommends gnupg lsb-release ca-certificates curl && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y libgz-tools2-dev && \
    rm -rf /var/lib/apt/lists/*

# defaults
ARG userid=1002
ARG groupid=1002

# Create or reuse the user safely
RUN existing_uid=$(getent passwd ${userid} | cut -d: -f1) && \
    if [ -n "$existing_uid" ]; then \
        echo "Reusing existing UID ${userid} (user: $existing_uid) as 'robot'"; \
        usermod -l robot -d /home/robot -m "$existing_uid"; \
    else \
        echo "Creating new user 'robot' with UID ${userid} and GID ${groupid}"; \
        groupadd -g ${groupid} -o robot || true; \
        useradd -ms /bin/bash --uid ${userid} -g ${groupid} --groups sudo,video robot; \
    fi && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers


USER robot
ENV HOME=/home/robot
WORKDIR /home/robot

# ROS_LOCALHOST_ONLY is deprecated
# LIBGL_ALWAYS_SOFTWARE=1 causes performance issues with Gazebo rendering
ENV USER=robot \
    LANG=en_US.UTF-8 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    HOME=/home/robot \
    XDG_RUNTIME_DIR=/run/user/${userid} \
    LIBGL_ALWAYS_SOFTWARE=0 \
    TZ=America/New_York

RUN sudo mkdir -p -m 0700 /run/user/${userid} && \
    sudo chown ${userid}:${groupid} /run/user/${userid}

# ros workspace
RUN source /opt/ros/jazzy/setup.bash \
    && mkdir -p ~/ros2_ws/src

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    # segmentation fault workaround
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc && \
    # autocompletion for Gazebo CLI
    echo "source /usr/share/bash-completion/completions/gz" >> ~/.bashrc && \
    # echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc && \
    echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> ~/.bashrc && \
    echo "alias clc=clear" >> ~/.bashrc && \
    # clear terminal when starting
    echo "clear" >> ~/.bashrc && \
    echo "toilet -f smblock --filter border:metal -w 24 ' ENEE 467     LAB 5        PART II'" >> ~/.bashrc

# Setup tmux config
ADD --chown=robot:robot https://raw.githubusercontent.com/MarylandRoboticsCenter/someConfigs/refs/heads/master/.tmux_K.conf /home/robot/.tmux.conf

ENTRYPOINT ["/bin/bash", "-i"]
