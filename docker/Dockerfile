FROM ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    wget \
    gnupg \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic
RUN wget -qO - https://packages.osrfoundation.org/gazebo.gpg | sudo apt-key add - && \
    echo "deb [arch=amd64,arm64] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    gazebo-harmonic \
    ros-jazzy-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# install VS Code server
RUN curl -fsSL https://code-server.dev/install.sh | sh

# install python extension
RUN code-server --install-extension ms-python.python

# Timezone setup
ARG TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo "${TZ}" > /etc/timezone

# Create non-root user
ARG userid=1002
ARG groupid=1002
RUN groupadd -g ${groupid} -o robot && \
    useradd -ms /bin/bash --uid ${userid} -g ${groupid} robot && \
    echo "robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER robot
WORKDIR /home/robot

# ROS 2 colcon workspace setup (common_ws)
COPY --chown=robot:robot lab-5.repos /tmp/lab-5.repos
RUN mkdir -p /home/robot/common_ws/src
WORKDIR /home/robot/common_ws
RUN vcs import --input /tmp/lab-5.repos .

# ROS 2 colcon workspace setup (lab-5)
RUN mkdir -p /home/robot/lab-5/src
WORKDIR /home/robot/lab-5

# Copy source tree into the image
COPY --chown=robot:robot src /home/robot/lab-5/src

# Initialize rosdep
RUN sudo rosdep init && rosdep update

# Install package dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

RUN chmod +x /home/robot/lab-5/src/robot_3r_planners/scripts
# Add to bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/robot/.bashrc && \
    echo "source /home/robot/lab-5/install/setup.bash" >> /home/robot/.bashrc && \
    echo "cd ~/lab-5" >> /home/robot/.bashrc

WORKDIR /home/robot/lab-5

# tmux config
ADD --chown=robot:robot \
    https://raw.githubusercontent.com/MarylandRoboticsCenter/someConfigs/refs/heads/master/.tmux_K.conf \
    /home/robot/.tmux.conf

# Default entry
ENTRYPOINT ["/bin/bash", "-i"]