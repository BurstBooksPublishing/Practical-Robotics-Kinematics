import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
import math

class DiffSplitter(Node):
    def __init__(self):
        super().__init__('diff_splitter')
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('half_track', 0.25)
        self.r = self.get_parameter('wheel_radius').value
        self.b = self.get_parameter('half_track').value
        # subscribe to cmd_vel from planner
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        # publish wheel angular velocities (right,left)
        self.wheel_pub = self.create_publisher(Float64MultiArray, 'wheel_vel_cmds', 10)
        # broadcast base->odom transform (example static integration here)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.x = self.y = self.theta = 0.0

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z
        # invert eq. (1) to wheel angular velocities
        w_r = (v + self.b * omega) / self.r
        w_l = (v - self.b * omega) / self.r
        arr = Float64MultiArray(data=[w_r, w_l])
        self.wheel_pub.publish(arr)  # hardware interface subscribes
        # integrate a simple dead-reckoning pose for tf (illustrative only)
        dt = 0.02
        delta_s = v * dt
        delta_theta = omega * dt
        self.x += delta_s * math.cos(self.theta + delta_theta/2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta/2.0)
        self.theta += delta_theta
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # set quaternion from yaw (z)
        qz = math.sin(self.theta/2.0); qw = math.cos(self.theta/2.0)
        t.transform.rotation.x = 0.0; t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz; t.transform.rotation.w = qw
        self.tf_broadcaster.send_transform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DiffSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()