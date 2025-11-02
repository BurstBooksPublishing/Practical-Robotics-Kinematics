# Dockerfile (use as ./Dockerfile)
FROM ros:noetic-ros-core  # small, pinned ROS base
ENV DEBIAN_FRONTEND=noninteractive

# Install OS-level deps (pin versions where possible)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-pip python3-venv \
    libeigen3-dev libboost-all-dev libgl1-mesa-glx \
    xvfb x11-utils && \
    rm -rf /var/lib/apt/lists/*

# Create non-root user for GUI and permissions
ARG USER=rosdev
ARG UID=1000
RUN useradd -m -u $UID -s /bin/bash $USER

# Install Python robotics libraries (pin versions)
RUN python3 -m pip install --no-cache-dir \
    pinocchio==2.4.0 pybullet==3.2.2 numpy==1.23.5

# Workspace copy and simple entry (adjust to project)
WORKDIR /home/$USER/ws
COPY ./src ./src  # copy kinematics code (small example)
RUN chown -R $USER:$USER /home/$USER/ws

USER $USER
ENV HOME=/home/$USER
CMD ["/bin/bash", "-lc", "source /opt/ros/noetic/setup.bash && bash"]