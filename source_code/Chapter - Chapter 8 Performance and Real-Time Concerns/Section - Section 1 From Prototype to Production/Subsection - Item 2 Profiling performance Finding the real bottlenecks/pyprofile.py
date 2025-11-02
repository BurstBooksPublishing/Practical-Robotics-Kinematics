import cProfile, pstats, io
import pinocchio as pin            # Pinocchio Python binding
model = pin.buildModelFromUrdf('robot.urdf')  # build model
data = model.createData()         # create data container
q = pin.neutral(model)            # neutral joint vector (example)

def control_cycle(n_iters=1000):
    for _ in range(n_iters):
        pin.forwardKinematics(model, data, q)  # FK (hotspot)
        pin.updateFramePlacements(model, data)
        pin.computeJointJacobians(model, data, q)  # Jacobians

pr = cProfile.Profile()
pr.enable()
control_cycle()                   # run real loop workload
pr.disable()
s = io.StringIO()
ps = pstats.Stats(pr, stream=s).sort_stats('cumulative')
ps.print_stats(20)                # top 20 cumulative entries
print(s.getvalue())               # inspect results