import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
# node prints transform matrix and age (sec)
class TfChecker(Node):
    def __init__(self):
        super().__init__('tf_checker')
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.timer = self.create_timer(0.5, self.check_cb)
        self.target = 'world'      # target frame
        self.source = 'camera_link' # source frame
    def check_cb(self):
        try:
            t: TransformStamped = self.buf.lookup_transform(
                self.target, self.source, rclpy.time.Time())
            now = self.get_clock().now()
            age = (now - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
            # build homogeneous matrix
            q = t.transform.rotation
            p = t.transform.translation
            # compute rotation matrix from quaternion (use tf_transformations or numpy here)
            import tf_transformations as tfm, numpy as np
            R = np.array(tfm.quaternion_matrix([q.x,q.y,q.z,q.w]))[:3,:3]
            T = np.eye(4); T[:3,:3]=R; T[:3,3]=[p.x,p.y,p.z]
            self.get_logger().info(f'Age={age:.3f}s T=\n{T}')
        except Exception as e:
            self.get_logger().warn(f'Lookup failed: {e}')
def main(args=None):
    rclpy.init(args=args); node=TfChecker(); rclpy.spin(node)