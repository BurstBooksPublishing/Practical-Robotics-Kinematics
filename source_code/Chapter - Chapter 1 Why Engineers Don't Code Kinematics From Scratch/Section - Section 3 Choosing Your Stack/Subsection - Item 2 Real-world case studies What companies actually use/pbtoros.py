import pybullet as p                # simulation FK
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('pb_fk_publisher')  # ROS node
pub = rospy.Publisher('/desired_ee_pose', PoseStamped, queue_size=1)

p.connect(p.DIRECT)                 # headless sim
robot = p.loadURDF('robot.urdf', useFixedBase=True)
ee_link = 7                          # example link index

rate = rospy.Rate(50)                # 50 Hz control loop
while not rospy.is_shutdown():
    # getLinkState returns (pos, orn, ...); pos tuple, orn quaternion tuple
    link_state = p.getLinkState(robot, ee_link, computeForwardKinematics=True)
    pos, orn = link_state[0], link_state[1]

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'base_link'
    msg.pose.position.x = pos[0]     # x
    msg.pose.position.y = pos[1]     # y
    msg.pose.position.z = pos[2]     # z
    msg.pose.orientation.x = orn[0]  # qx
    msg.pose.orientation.y = orn[1]  # qy
    msg.pose.orientation.z = orn[2]  # qz
    msg.pose.orientation.w = orn[3]  # qw

    pub.publish(msg)                 # MoveIt or planner consumes topic
    rate.sleep()