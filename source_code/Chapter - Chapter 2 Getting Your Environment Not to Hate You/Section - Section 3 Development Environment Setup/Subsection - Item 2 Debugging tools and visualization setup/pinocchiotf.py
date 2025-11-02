import rospy
import pinocchio as pin
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# load model (URDF path) and data
model = pin.buildModelFromUrdf('robot.urdf')        # load robot URDF
data = model.createData()
ee_frame = 'ee_link'

rospy.init_node('pinocchio_tf_broadcaster')
br = TransformBroadcaster()
rate = rospy.Rate(50)  # 50 Hz visualization

q = pin.neutral(model)  # initial joint config
while not rospy.is_shutdown():
    # update q from sensors or simulation here
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    oMf = data.oMf[model.getFrameId(ee_frame)]  # homogeneous matrix

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'base_link'
    t.child_frame_id = ee_frame
    # fill translation and rotation (quaternion)
    t.transform.translation.x = oMf.translation[0]
    t.transform.translation.y = oMf.translation[1]
    t.transform.translation.z = oMf.translation[2]
    qx, qy, qz, qw = pin.Quaternion(oMf.rotation).coeffs()  # note order may vary
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw

    br.sendTransform(t)  # publish for rviz
    rate.sleep()