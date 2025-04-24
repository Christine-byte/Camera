# Set the ROS distribution as an argument (default: jazzy)
ARG ROS_DISTRO=jazzy

# Use official ROS 2 base image
FROM ros:${ROS_DISTRO}-ros-base

# Set working directory
WORKDIR /depthai_ros_workspace

# Set bash as the default shell, so we can se pushd, source and other commands.
SHELL ["/bin/bash", "-c"]

# Prevent prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# TODO Required for privacy-MUST UNCOMMENT
#ENV ROS_DOMAIN_ID=3

# Update system and install required packages
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
    # Basic Tools for Camera
    git \
    nano \
    wget \
    unzip \
    python3-pip \
    python3-venv \
    python3-rosdep \
    # GUI Tools
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rqt-reconfigure \
    ros-${ROS_DISTRO}-rqt-publisher \
    ros-${ROS_DISTRO}-rqt-console \
    # Provide what the Qt graphics platform needs to run
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-xkb1 \
    libqt5widgets5 \
    libqt5gui5 \
    libqt5core5a \
    qtbase5-dev \
    # System Libraries: Supports USB, Image, OpenCV
    libusb-1.0-0 \
    libglib2.0-0 \
    libopencv-dev \
    libboost-dev \ 
    # ROS Image Pipeline related (basic)
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-vision-msgs \
    # Advanced features for ROS image processing
    ros-jazzy-image-transport-plugins \
    ros-jazzy-image-pipeline \
    ros-jazzy-depth-image-proc \
    ros-jazzy-camera-calibration \
    # ROS supports plugins and messages
    ros-jazzy-xacro \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-ffmpeg-image-transport-msgs \
    ros-jazzy-foxglove-msgs \
    ros-jazzy-rviz-imu-plugin \
    # DepthAI component (optional if using prebuilt)
    ros-jazzy-depthai \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport && \
    rm -rf /var/lib/apt/lists/*  # Clean up to reduce image size

# Create udev rules directory (prevent tee write failure in script)
RUN mkdir -p /etc/udev/rules.d

# Install dependencies - Depthai-Core (C++ SDK),udev rules (making OAK camera accessible to regular users), depthai Python SDK will be installed automatically
RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | bash

# Initialize rosdep for installing ROS package dependencies
RUN [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ] && rosdep init || echo "rosdep already initialized" && rosdep update

# create ROS2 workspace and clone depthai-ros source code
WORKDIR /depthai_ros_workspace/src
RUN git clone --branch ${ROS_DISTRO} https://github.com/luxonis/depthai-ros.git

# Install ROS dependencies and build
WORKDIR /depthai_ros_workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --parallel-workers 1
    
# The ROS2 environment variable is automatically loaded, and the "ros2" command can be used directly in the container
RUN echo "source /opt/ros/jazzy/setup.bash && source /depthai_ros_workspace/install/setup.bash" >> /root/.bashrc

# Default command to keep the container running (can be overridden)
CMD ["/bin/bash"]
