import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.optimize import least_squares
import numpy as np

# Build robot from URDF (reuse simulation URDF + meshes)
robot = RobotWrapper.BuildFromURDF('path/to/robot.urdf', ['path/to/package'])
model = robot.model
data = model.createData()

# Initial configuration and frame selection
q0 = pin.neutral(model)                   # reasonable seed from model
ee_frame = 'ee_link'                      # match PyBullet frame name
ee_id = model.getFrameId(ee_frame)

# joint bounds (use model limits if present)
lb = np.array([jl.lower for jl in model.lowerPositionLimit])  # use arrays from model
ub = np.array([ju.upper for ju in model.upperPositionLimit])

# desired task-space position from higher-level planner or simulation
target_pos = np.array([0.6, 0.0, 0.2])

def fk_position(q):
    pin.forwardKinematics(model, data, q)      # compute joint transforms
    pin.updateFramePlacements(model, data)     # compute data.oMf for frames
    return data.oMf[ee_id].translation        # numpy (3,) position

def residual(q):
    q = np.asarray(q)
    pos = fk_position(q)
    return pos - target_pos                   # 3-d residual

# solve with bounds; use trust region reflective (least_squares handles bounds)
res = least_squares(residual, q0, bounds=(lb, ub), xtol=1e-6, ftol=1e-6)

q_sol = res.x
# validate solution: FK and simple collision check integration point
ee_sol = fk_position(q_sol)
print("Solved ee pos:", ee_sol)              # quick sanity check