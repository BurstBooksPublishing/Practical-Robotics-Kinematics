#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

rospy.init_node('traj_client')  # node for action client
client = actionlib.SimpleActionClient(
    '/arm_controller/follow_joint_trajectory',  # controller action name
    FollowJointTrajectoryAction)
client.wait_for_server()

# build trajectory goal
traj = JointTrajectory()
traj.joint_names = ['joint1','joint2','joint3']  # joint order must match controller
p = JointTrajectoryPoint(positions=[0.5, 0.0, -0.3], velocities=[0,0,0], time_from_start=rospy.Duration(2.0))
traj.points.append(p)

goal = FollowJointTrajectoryGoal()
goal.trajectory = traj

client.send_goal(goal)  # non-blocking; monitors via client.wait_for_result()
client.wait_for_result(rospy.Duration(5.0))
# brief logging
rospy.loginfo('Trajectory result: %s' % str(client.get_result()))