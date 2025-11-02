#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import moveit_commander
# init MoveIt/ROS (assumes ROS node initialized)
moveit_commander.roscpp_initialize([])
robot = moveit_commander.RobotCommander()          # access robot model & state
group = moveit_commander.MoveGroupCommander("manipulator")  # your group name

tf_bcast = tf2_ros.TransformBroadcaster()
current_js = {}  # cache joint positions

def joint_state_cb(msg):                             # update cache on /joint_states
    for n,v in zip(msg.name, msg.position):
        current_js[n] = v
    publish_link_poses()

def publish_link_poses():
    # set RobotState from joint cache for the group's joints
    # get_current_joint_values() uses internal state if available; we prefer explicit set
    joint_names = group.get_active_joints()          # names exactly as in URDF
    joint_values = [current_js.get(n, 0.0) for n in joint_names]
    # set the group's joint positions in robot state
    robot_state = robot.get_current_state()          # returns moveit_msgs/RobotState
    # convert robot_state to moveit_commander RobotState API for FK (internal)
    # In moveit_commander, use set_joint_value_target on group and then compute FK via get_current_pose
    group.set_joint_value_target(joint_values)
    # compute poses for two links (branched arms or two end-effectors)
    for link_name in ["left_ee_link", "right_ee_link"]:  # replace with your link names
        pose = group.get_current_pose(end_effector_link=link_name).pose  # uses RobotState FK
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = robot.get_planning_frame()  # world/planning frame
        t.child_frame_id = link_name + "_tf"
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        tf_bcast.sendTransform(t)  # publish for visualization in rviz
        # minimal inline comment: TF allows other nodes to consume the computed pose

if __name__ == "__main__":
    rospy.init_node("moveit_fk_publisher")
    rospy.Subscriber("/joint_states", JointState, joint_state_cb, queue_size=1)
    rospy.spin()