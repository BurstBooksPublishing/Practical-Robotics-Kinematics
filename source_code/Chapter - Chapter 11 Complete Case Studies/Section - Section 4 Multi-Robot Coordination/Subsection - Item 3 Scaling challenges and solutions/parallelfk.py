import pinocchio as pin
from concurrent.futures import ThreadPoolExecutor
# robot_models: list of (model, data) for each robot
def compute_fk(model_data_q):
    model, data, q = model_data_q
    pin.forwardKinematics(model, data, q)            # compute transforms
    pin.updateFramePlacements(model, data)           # update frame placements
    return data.oMf[model.getFrameId("end_effector")]  # return EE placement

with ThreadPoolExecutor(max_workers=8) as ex:
    results = list(ex.map(compute_fk, robot_models))  # parallel FK calls
# results: list of end-effector placements for downstream planning