from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.autodiffutils import InitializeAutoDiff  # helper to seed AutoDiff
import numpy as np

plant = MultibodyPlant(time_step=0.0)
Parser(plant).AddModelFromFile("arm.urdf")  # real URDF for your robot
plant.Finalize()

# Current numeric state (numpy arrays)
q = np.array([ ... ])      # joint positions (length n)
qdot = np.array([ ... ])   # joint velocities (length n)

# 1) Compute numeric Jacobian (double)
context = plant.CreateDefaultContext()
context.SetPositions(q)
J_num = plant.CalcJacobianTranslationalVelocity(
    context,
    plant.world_frame(),    # with_respect_to_frame
    plant.GetFrameByName("ee_link"),  # frame_B: end-effector frame
    [0.0, 0.0, 0.0],        # p_BoBq: point on EE (origin)
    plant.world_frame())    # expressed_in_frame
# J_num is a 3 x n numeric Jacobian (translational part)

# 2) Compute Jdot * qdot via AutoDiff directional derivative
# Seed positions with derivatives equal to qdot to get directional derivative.
q_ad = InitializeAutoDiff(q, qdot)         # AutoDiff vector with derivative = qdot
plant_ad = plant.ToAutoDiffXd()
context_ad = plant_ad.CreateDefaultContext()
context_ad.SetPositions(q_ad)               # positions are AutoDiffXd
# velocities can be set numerically; not required for J evaluation
J_ad = plant_ad.CalcJacobianTranslationalVelocity(
    context_ad,
    plant_ad.world_frame(),
    plant_ad.GetFrameByName("ee_link"),
    [0.0, 0.0, 0.0],
    plant_ad.world_frame())
# Each entry in J_ad is AutoDiffXd. Its derivative equals (Jdot*qdot) entry.
Jdot_qdot = np.array([[elem.derivatives()[0] for elem in row] for row in J_ad])
# Jdot_qdot is 3 x n, equal to (d/dt J) * qdot

# Note: For full Jdot tensor, seed InitializeAutoDiff with identity columns and assemble.