import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
from collections import deque

class RealtimeJointPublisher(Node):
    def __init__(self):
        super().__init__('realtime_joint_pub')
        # Best effort keeps latency low for high-rate sensors; Reliable for critical events.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.pub = self.create_publisher(JointState, '/joint_states', qos)
        self.buffer = deque(maxlen=1000)  # small ring buffer for producer/consumer
        self.timer = self.create_timer(0.004, self._timer_cb)  # 250 Hz publish loop

    def feed_joint_samples(self, joint_names, positions):
        # called by non-blocking data collection thread; push to buffer
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = joint_names
        js.position = positions
        self.buffer.append(js)

    def _timer_cb(self):
        # publish the latest sample if available; otherwise do nothing (no blocking).
        if self.buffer:
            msg = self.buffer.pop()  # drop older samples to favor recent state
            self.pub.publish(msg)
            # optional: log only if latency exceeds threshold
            now = self.get_clock().now().to_msg()
            sent = msg.header.stamp
            latency_ms = (rclpy.time.Time.from_msg(now) - rclpy.time.Time.from_msg(sent)).nanoseconds / 1e6
            if latency_ms > 50:
                self.get_logger().warn(f'Visualization latency {latency_ms:.1f} ms')
                
def main(args=None):
    rclpy.init(args=args)
    node = RealtimeJointPublisher()
    try:
        # In a real system, feed_joint_samples is called from your hardware thread.
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()