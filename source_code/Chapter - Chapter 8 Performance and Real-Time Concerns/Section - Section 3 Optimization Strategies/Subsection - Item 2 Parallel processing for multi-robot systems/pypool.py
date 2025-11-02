import pybullet as p
from concurrent.futures import ProcessPoolExecutor
import numpy as np

# per-process globals (initialized once)
_robot_body = None
_link_index = 7  # example end-effector link index

def _init_worker(urdf_path):
    # connect once per process and load robot model (cheap thereafter)
    global _robot_body
    p.connect(p.DIRECT)               # headless worker
    _robot_body = p.loadURDF(urdf_path, useFixedBase=True)

def compute_kinematics(q):
    # q: list or numpy array of joint positions
    # returns end-effector pose and Jacobian (linear, angular)
    global _robot_body, _link_index
    zeros = [0.0] * len(q)
    local_pos = [0., 0., 0.]  # point on link
    jac_t, jac_r = p.calculateJacobian(
        _robot_body, _link_index, local_pos, list(q), zeros, zeros)
    pos, orn = p.getLinkState(_robot_body, _link_index)[:2]
    return {'pos': pos, 'orn': orn, 'jac_lin': np.array(jac_t), 'jac_ang': np.array(jac_r)}

# example driver: parallel compute for many robots' states
if __name__ == '__main__':
    urdf = 'urdf/my_robot.urdf'
    states = [np.zeros(7) for _ in range(100)]  # 100 queries
    with ProcessPoolExecutor(max_workers=4, initializer=_init_worker, initargs=(urdf,)) as ex:
        results = list(ex.map(compute_kinematics, states))
    # results now contains FK and Jacobian for each state