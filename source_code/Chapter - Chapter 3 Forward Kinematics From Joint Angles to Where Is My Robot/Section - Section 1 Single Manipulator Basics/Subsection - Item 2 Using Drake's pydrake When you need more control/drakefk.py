from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsers import Parser
from pydrake.math import RigidTransform
import numpy as np

# Build plant and parser (zero time-step for continuous model).
plant = MultibodyPlant(time_step=0.0)
parser = Parser(plant)
model = parser.AddModelFromFile("urdfs/industrial_arm.urdf")  # URDF path
plant.Finalize()  # must call before creating Context

context = plant.CreateDefaultContext()

# Prepare a joint vector q with length plant.num_positions().
# Fill with a realistic nominal pose (radians for revolute joints).
q = np.array([0.0, -1.2, 0.8, 0.0, 1.0, 0.2])  # length must match num_positions
plant.SetPositions(context, q)  # set generalized positions in Context

# Get frames: world_frame() and the end-effector frame by name.
world = plant.world_frame()
ee_frame = plant.GetFrameByName("ee_link")  # use the frame name from URDF

# Compute transform: world -> ee.
T_world_ee = plant.CalcRelativeTransform(context, world, ee_frame)

# Extract translation and quaternion for downstream controllers or logging.
translation = T_world_ee.translation()          # numpy-like access
rotation_quat = T_world_ee.rotation().ToQuaternion()  # returns Quaternion
print("EE position (m):", translation)
print("EE quat (w, x, y, z):", rotation_quat.w(), rotation_quat.x(),
      rotation_quat.y(), rotation_quat.z())