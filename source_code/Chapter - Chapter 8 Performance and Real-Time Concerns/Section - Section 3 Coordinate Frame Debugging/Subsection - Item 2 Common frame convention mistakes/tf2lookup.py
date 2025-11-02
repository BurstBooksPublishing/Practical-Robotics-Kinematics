import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
# assume node has been initialized elsewhere

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# COMMON MISTAKE: reversing target and source (gives inverse transform).
# WRONG: transform = tf_buffer.lookup_transform('ee_link','base_link', rospy.Time(0))

# CORRECT: target frame is the frame you want data expressed in,
# source frame is where the original coordinates live.
try:
    # express 'ee_link' pose in 'base_link' coordinates (correct order)
    transform = tf_buffer.lookup_transform('base_link', 'ee_link', rospy.Time(0),
                                           rospy.Duration(1.0))  # wait up to 1s
    # transform.transform.translation, transform.transform.rotation available
except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
    rospy.logerr("Transform lookup failed: %s", e)
# Always normalize quaternions before using them in downstream libraries.
# Example: norm = sqrt(x*x + y*y + z*z + w*w); divide components if norm deviates.