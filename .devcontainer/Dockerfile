FROM osrf/ros:humble-desktop-full

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    clang-format \
    git \
    vim \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    flake8 \
    autopep8 \
    pytest \
    numpy \
    scipy

# Create workspace
RUN mkdir -p /workspace/ros2_ws/src

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# Create non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Setup user environment
USER $USERNAME
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

WORKDIR /workspace

