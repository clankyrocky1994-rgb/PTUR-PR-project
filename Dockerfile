FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV FRI_CLIENT_VERSION=1.15
ENV PIP_DEFAULT_TIMEOUT=1000
ENV PIP_RETRIES=10

SHELL ["/bin/bash", "-c"]

# Install system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    pkg-config \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/*

# Install common ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2 \
    ros-jazzy-moveit \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-realtime-tools \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy project files into the container
COPY . /workspace

# Import KUKA LBR FRI dependencies if the LBR stack submodule is present
RUN if [ -f src/lbr_fri_ros2_stack/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml ]; then \
      vcs import src < src/lbr_fri_ros2_stack/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml; \
    fi

# Install ROS dependencies if possible
RUN rosdep update || true
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build ROS workspace.
# Do NOT use --symlink-install here, because it can trigger "--editable not recognized".
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build || true

# Install vision app dependencies into isolated virtual environment
RUN python3 -m venv /opt/vision_venv && \
    source /opt/vision_venv/bin/activate && \
    python -m pip install --upgrade pip setuptools wheel && \
    if [ -f robot_vision_app/requirements.txt ]; then \
      pip install \
        --default-timeout=1000 \
        --retries 10 \
        --no-cache-dir \
        -r robot_vision_app/requirements.txt; \
    fi

# Source ROS automatically and activate vision venv when container starts
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc && \
    echo "alias vision_env='source /opt/vision_venv/bin/activate'" >> /root/.bashrc

CMD ["/bin/bash"]