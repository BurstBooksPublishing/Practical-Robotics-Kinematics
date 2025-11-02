#!/usr/bin/env bash
set -euo pipefail

# Configuration (edit once per project)
ROS_DISTRO="humble"                  # ROS distribution
WORKSPACE="$HOME/ros_ws_project"     # workspace root
REPOS_FILE="$WORKSPACE/ros2.repos"   # pinned repo list

# Ensure system rosdep db is initialized (run as user once)
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-vcstool \
                   python3-rosdep git # required host tools
sudo rosdep init || true
rosdep update

# Create workspace layout
mkdir -p "$WORKSPACE/src"
cd "$WORKSPACE"

# Example pinned repos file (create if missing); use exact SHA for determinism
if [ ! -f "$REPOS_FILE" ]; then
  cat > "$REPOS_FILE" <<'YAML'
repositories:
  moveit2:
    type: git
    url: https://github.com/ros-planning/moveit2.git
    version: 3f1b2a1f3f5e3c1c2a4b5d6e7f8a9b0c1d2e3f4a
  pinocchio_bindings:
    type: git
    url: https://github.com/stack-of-tasks/pinocchio.git
    version: b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1
YAML
fi

# Import pinned repositories to src/
python3 -m vcstool import src < "$REPOS_FILE"   # reproducible clone by SHA

# Install system dependencies for all packages in src/ via rosdep
rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"

# Create a Python venv for site-package isolation (useful for PyPI kinematics libs)
python3 -m venv "$WORKSPACE/venv"               # create venv
source "$WORKSPACE/venv/bin/activate"
pip install -U pip setuptools wheel             # minimal trusted toolchain
pip install -r src/my_python_pkg/requirements.txt || true  # project optional

# Build deterministic install layout
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace for immediate use
source install/setup.bash
echo "Workspace ready at $WORKSPACE (source install/setup.bash to use)"