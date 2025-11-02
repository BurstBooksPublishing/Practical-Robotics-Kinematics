#!/usr/bin/env bash
set -euo pipefail

# 1) Create conda env (recommended for Pinocchio)
conda create -n robot-kin python=3.10 -y  # choose a supported Python
conda activate robot-kin

# 2) Install Pinocchio and essential C++ deps from conda-forge
conda install -c conda-forge pinocchio eigen pybind11 -y  # prebuilt C++ bindings

# 3) Use pip for pure-python or wheel packages
python -m pip install --upgrade pip setuptools wheel  # ensure modern pip
python -m pip install pybullet                      # PyBullet wheel

# 4) Try installing Drake's python API; fall back to Docker if pip fails
if python -c "import importlib, sys; importlib.import_module('pydrake')" >/dev/null 2>&1; then
  echo "pydrake already available"
else
  python -m pip install pydrake || \
    echo "pydrake pip install failed; use Drake docker or OS packages per Drake docs"
fi

# 5) Quick smoke tests (Python snippet)
python - <<'PY'
# quick imports to validate installation -- replace 'examples/robot.urdf' with your URDF
import sys
try:
    import pybullet as p                # physics simulator
    print("pybullet OK, client:", p.connect(p.DIRECT))
except Exception as e:
    print("pybullet import failed:", e, file=sys.stderr)

try:
    import pinocchio as pin             # kinematics library
    print("pinocchio OK, version:", pin.__version__)
except Exception as e:
    print("pinocchio import failed:", e, file=sys.stderr)

try:
    from pydrake.all import Parser     # tentative Drake import
    print("pydrake OK")
except Exception as e:
    print("pydrake import failed (use Docker/apt):", e, file=sys.stderr)
PY