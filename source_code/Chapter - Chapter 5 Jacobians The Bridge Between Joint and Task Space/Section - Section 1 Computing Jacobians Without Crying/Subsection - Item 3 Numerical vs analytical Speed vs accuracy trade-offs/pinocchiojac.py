import pinocchio as pin
  # model, data created from URDF earlier; q is a numpy vector
  pin.forwardKinematics(model, data, q)                # compute transforms
  pin.updateFramePlacements(model, data)               # ensure frame placements
  pin.computeJointJacobians(model, data, q)            # fills joint jacobians
  J = pin.getFrameJacobian(model, data, frame_id,     # world-frame Jacobian
                            pin.ReferenceFrame.WORLD)
  # J is 6x n: spatial velocity jacobian for frame_id