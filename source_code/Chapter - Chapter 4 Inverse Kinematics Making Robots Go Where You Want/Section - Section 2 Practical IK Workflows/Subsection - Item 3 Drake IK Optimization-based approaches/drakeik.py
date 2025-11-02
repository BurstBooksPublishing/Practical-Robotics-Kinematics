from pydrake.all import (DiagramBuilder, MultibodyPlant, SceneGraph,
                         Parser, InverseKinematics, Solve)
# Build plant and load robot URDF.
builder = DiagramBuilder()
plant = MultibodyPlant(time_step=0.0)
scene_graph = builder.AddSystem(SceneGraph())
plant.RegisterAsSourceForSceneGraph(scene_graph)
Parser(plant).AddModelFromFile("franka_panda/panda.urdf")  # robot URDF
plant.Finalize()

# Choose the end-effector frame.
ee_frame = plant.GetFrameByName("panda_link8")  # end-effector frame

# Construct InverseKinematics with a seed; seed helps solver initialization.
q_seed = plant.GetPositions(plant.GetJointIndices()) * 0.0  # example seed
ik = InverseKinematics(plant, q_seed)

# Add a position constraint (target expressed in world frame).
target_point_in_ee = [0., 0., 0.05]  # offset in EE frame
target_position_world = [0.6, 0.0, 0.4]
ik.AddPositionConstraint(ee_frame, target_point_in_ee,
                         plant.world_frame(), target_position_world,
                         target_position_world)  # equality (tight bounds)

# Add an orientation constraint (tight quaternion tolerance).
R_des = ...  # 3x3 rotation matrix for desired orientation
ik.AddOrientationConstraint(ee_frame, R_des, plant.world_frame(), R_des, 1e-3)

# Optional: add quadratic cost to keep near q_seed.
W = 1e-2 * np.eye(plant.num_positions())  # small regularization
ik.prog().AddQuadraticCost(W, q_seed, ik.q())  # keeps solution near seed

# Solve and extract joint angles.
result = Solve(ik.prog())                # select default solver
if result.is_success():
    q_sol = result.GetSolution(ik.q())   # joint solution vector
else:
    raise RuntimeError("Drake IK failed to converge")