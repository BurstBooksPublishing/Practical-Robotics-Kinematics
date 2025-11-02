import time, statistics, numpy as np

def run_benchmark(adapter, q_samples, n_warm=50):
    # adapter must implement: init(), fk(q)->pose, jacobian(q)->np.array
    adapter.init()                      # library-specific loading
    # warm-up to remove JIT / cache effects
    for q in q_samples[:n_warm]:
        adapter.fk(q); adapter.jacobian(q)
    lat_fk, lat_jac = [], []
    for q in q_samples[n_warm:]:
        t0 = time.perf_counter()
        adapter.fk(q)
        t1 = time.perf_counter()
        adapter.jacobian(q)
        t2 = time.perf_counter()
        lat_fk.append((t1-t0)*1e3)    # ms
        lat_jac.append((t2-t1)*1e3)   # ms
    # summary stats
    def summary(xs):
        return {'mean': statistics.mean(xs),
                'std': statistics.pstdev(xs),
                'p99': np.percentile(xs,99.0),
                'max': max(xs)}
    return {'fk': summary(lat_fk), 'jacobian': summary(lat_jac)}

# Adapter note: for Pinocchio implement fk via pinocchio.forwardKinematics
# and getFramePlacement, jacobian via pinocchio.computeJointJacobians.
# For Drake use plant.CalcRelativeTransform and plant.CalcJacobianSpatialVelocity.