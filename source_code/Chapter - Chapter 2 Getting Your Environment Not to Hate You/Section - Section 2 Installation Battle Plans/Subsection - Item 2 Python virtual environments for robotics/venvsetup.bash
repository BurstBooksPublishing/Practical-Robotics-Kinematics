# install system prerequisites (run as root) -- Eigen, BLAS, CMake, build tools
apt-get update && apt-get install -y build-essential cmake libeigen3-dev libblas-dev liblapack-dev

# use a known Python minor version for binary wheels
python3.8 -m venv ./venv            # create venv
source ./venv/bin/activate         # activate venv

# upgrade pip and use a constraints file for determinism
pip install --upgrade pip setuptools wheel
# constraints.txt pins transitive dependency versions (generated via pip-compile)
pip install -r requirements.txt -c constraints.txt

# quick verification script (import native bindings)
python - <<'PY'                       # inline python test
import pinocchio                        # quick import test for Pinocchio
import pybullet as pgb                  # test PyBullet import
print("Pinocchio", pinocchio.__version__)  # runtime validation
print("PyBullet", pgb.__version__)
PY