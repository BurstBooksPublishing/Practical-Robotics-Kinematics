# minimal ROS2 example; adapt topic names and message fields to your stack
import math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class DiffDriveOdom(Node):
    def __init__(self):
        super().__init__('diff_drive_odom')
        self.declare_parameters(
            namespace='',
            parameters=[('wheel_radius', 0.05), ('wheel_base', 0.3)])
        self.r = self.get_parameter('wheel_radius').value  # meters
        self.b = self.get_parameter('wheel_base').value    # meters
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.prev_left = None; self.prev_right = None
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(JointState, 'joint_states', self.js_cb, 50)

    def js_cb(self, msg: JointState):
        # find joint indices by name; names commonly contain "left_wheel_joint"
        try:
            iL = msg.name.index('left_wheel_joint')
            iR = msg.name.index('right_wheel_joint')
        except ValueError:
            return  # ignore if names not present
        left = msg.position[iL]; right = msg.position[iR]  # radians
        now = self.get_clock().now()
        if self.prev_left is None:
            self.prev_left, self.prev_right, self.prev_t = left, right, now
            return
        dt = (now - self.prev_t).nanoseconds * 1e-9
        if dt <= 0: return
        dleft = left - self.prev_left
        dright = right - self.prev_right
        # wheel angular rates (rad/s)
        wL = dleft / dt; wR = dright / dt
        # body velocities using Eq. (1)
        v = self.r * 0.5 * (wR + wL)
        omega = self.r * (wR - wL) / self.b
        # pose integration using exact formula when |omega| > eps
        eps = 1e-6
        if abs(omega) > eps:
            dx = (v/omega) * (math.sin(self.th + omega*dt) - math.sin(self.th))
            dy = -(v/omega) * (math.cos(self.th + omega*dt) - math.cos(self.th))
        else:
            dx = v * math.cos(self.th) * dt
            dy = v * math.sin(self.th) * dt
        self.x += dx; self.y += dy; self.th += omega*dt
        # publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg(); odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y
        # quaternion around z: qz = sin(th/2), qw = cos(th/2)
        odom.pose.pose.orientation.z = math.sin(self.th*0.5)
        odom.pose.pose.orientation.w = math.cos(self.th*0.5)
        odom.twist.twist.linear.x = v; odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)
        # broadcast TF odom->base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg(); t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x; t.transform.translation.y = self.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)
        # save for next
        self.prev_left, self.prev_right, self.prev_t = left, right, now

def main(args=None):
    rclpy.init(args=args); node = DiffDriveOdom(); rclpy.spin(node); rclpy.shutdown()