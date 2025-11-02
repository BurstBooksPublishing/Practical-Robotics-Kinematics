# fk_backend.py
import os
USE_PINOCCHIO = os.getenv("FK_BACKEND","pybullet")=="pinocchio"

if USE_PINOCCHIO:
    import pinocchio as pin
    from pinocchio.robot_wrapper import RobotWrapper
    # build model once (URDF path required)
    ROBOT = RobotWrapper.BuildFromURDF("robot.urdf", package_dirs=["./packages"])
else:
    import pybullet as p
    # assume client and robot already loaded in PyBullet
    PBODY = 1  # example bodyUniqueId

def fk_end_effector(q, ee_name="ee_link"):
    """Return pose (pos, quat) of end-effector for joint vector q."""
    if USE_PINOCCHIO:
        ROBOT.model.upperPositionLimit  # tiny sanity check
        ROBOT.forwardKinematics(q)                     # compute placements
        pin.updateFramePlacements(ROBOT.model, ROBOT.data)
        fid = ROBOT.model.getFrameId(ee_name)
        oMf = ROBOT.data.oMf[fid]                       # SE3 pose
        pos = oMf.translation
        quat = pin.Quaternion(oMf.rotation).coeffs()    # (x,y,z,w) convention
        return pos, quat
    else:
        # apply joint states to PyBullet; getLinkState returns world pose
        for i, qi in enumerate(q):
            p.resetJointState(PBODY, i, qi)            # set joint without dynamics
        ls = p.getLinkState(PBODY, linkIndex=ee_link_index, computeForwardKinematics=True)
        return ls[4], ls[5]                            # worldLinkFramePosition, orientation