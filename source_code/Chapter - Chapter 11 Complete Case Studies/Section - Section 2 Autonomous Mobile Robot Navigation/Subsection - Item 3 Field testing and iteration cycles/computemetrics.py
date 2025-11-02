import rosbag
import numpy as np
from nav_msgs.msg import Path, Odometry

def point_to_segment_dist(p, a, b):
    # p,a,b are (x,y). returns perpendicular distance to segment ab.
    ap = np.array(p)-np.array(a); ab = np.array(b)-np.array(a)
    ab_len2 = np.dot(ab,ab)
    if ab_len2 == 0: return np.linalg.norm(ap)
    t = max(0, min(1, np.dot(ap,ab)/ab_len2))
    proj = np.array(a) + t*ab
    return np.linalg.norm(np.array(p)-proj)

bag = rosbag.Bag('field_run.bag')
planned = None
# read planned path (assumes single Path message)
for topic,msg,t in bag.read_messages(topics=['/planned_path']):
    planned = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
    break
# collect odometry
odom_xy = []
for topic,msg,t in bag.read_messages(topics=['/odom']):
    odom_xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

# compute cross-track errors
errs = []
for p in odom_xy:
    # nearest distance to any segment of planned path
    dmin = min(point_to_segment_dist(p, planned[i], planned[i+1])
               for i in range(len(planned)-1))
    errs.append(dmin)
bag.close()

rms_ct = np.sqrt(np.mean(np.array(errs)**2))
print('RMS cross-track (m):', rms_ct)  # use thresholds in engineering checklist