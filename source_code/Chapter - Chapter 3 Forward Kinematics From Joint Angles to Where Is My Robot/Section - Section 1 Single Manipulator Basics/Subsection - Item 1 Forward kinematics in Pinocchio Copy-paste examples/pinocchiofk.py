import pinocchio as pin
import numpy as np

try:
    import example_robot_data as erd
    robot = erd.load('panda')             # packaged model; adjust name if needed
    model, data = robot.model, robot.data # RobotWrapper provides prebuilt model/data
    q = robot.q0.copy()                   # sensible default joint configuration
except Exception:
    model = pin.buildModelFromUrdf('/path/to/robot.urdf')  # replace with actual path
    data = model.createData()
    q = pin.utils.zero(model.nq)          # zero seed for fixed-base manipulators

# compute forward kinematics (fills data)
pin.forwardKinematics(model, data, q)     # compute placements and joint-dependent info
pin.updateFramePlacements(model, data)    # update data.oMf for frames

frame_name = 'ee_link'                    # change to your URDF frame name
frame_id = model.getFrameId(frame_name)   # throws if frame not found

oMf = data.oMf[frame_id]                  # SE3 placement object
T = oMf.homogeneous()                     # 4x4 numpy homogeneous transform
print('End-effector homogeneous transform:\n', T)
print('Position (x,y,z):', oMf.translation)  # 3-vector position
# print('Rotation (3x3):\n', oMf.rotation)   # optional: 3x3 rotation matrix