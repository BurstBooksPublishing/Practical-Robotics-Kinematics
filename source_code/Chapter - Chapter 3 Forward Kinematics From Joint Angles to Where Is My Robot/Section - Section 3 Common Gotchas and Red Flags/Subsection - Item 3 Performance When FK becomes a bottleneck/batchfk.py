import multiprocessing as mp
import pinocchio as pin
import numpy as np

# worker performs FK on one q, returns end-effector SE3
def worker(args):
    model, q = args
    data = model.createData()            # one data per worker (thread-safe)
    pin.forwardKinematics(model, data, q)      # compute joint placements
    pin.updateFramePlacements(model, data)     # update frame placements
    ee_frame = model.getFrameId("ee")    # frame name example
    return data.oMf[ee_frame]            # return SE3 (placement)

def batch_fk(model, Q):
    # model: pinocchio Model instance; Q: (M, n) array of configurations
    with mp.Pool(processes=mp.cpu_count()) as pool:
        args = [(model, q) for q in Q]
        results = pool.map(worker, args)  # parallel map
    return results  # list of SE3 placements