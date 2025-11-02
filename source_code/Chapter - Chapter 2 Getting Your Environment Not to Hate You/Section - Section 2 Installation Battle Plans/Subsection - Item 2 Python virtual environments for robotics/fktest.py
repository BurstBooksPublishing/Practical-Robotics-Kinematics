# minimal pinocchio FK sanity check
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper  # quick robot loader

# load URDF from a realistic robot package installed in workspace
robot = RobotWrapper.BuildFromURDF("path/to/urdf/robot.urdf")  # comment: provide workspace path
q = pin.neutral(robot.model)                              # neutral joint configuration
oMf = robot.framePlacement(robot.model.getFrameId("ee_link"), q)  # end-effector pose
print("EE position:", oMf.translation)                    # verify no crash and plausible output