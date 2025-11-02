import threading, queue, time, json
import rclpy
from rclpy.node import Node
import pinocchio as pin
import numpy as np
from prometheus_client import Gauge, start_http_server

# Prometheus metrics (process-local exporter)
GAUGE_RESIDUAL = Gauge('kin_ik_residual', 'IK residual norm')
GAUGE_SIGMA_MIN = Gauge('kin_jacobian_sigma_min', 'Jacobian smallest singular value')
GAUGE_COMPUTE_MS = Gauge('kin_compute_time_ms', 'Kinematics compute latency ms')
COUNTER_FAIL = Gauge('kin_solver_failures_total', 'Solver failures count')  # use Counter in production

class KinematicsMonitor(Node):
    def __init__(self, robot_model, metric_port=8000):
        super().__init__('kin_monitor')
        self.model = robot_model
        self.data = robot_model.createData()
        self.pub = self.create_publisher(String, '/kinematics/metrics', 10)  # structured JSON
        self.q = np.zeros(self.model.nq)  # current joint state snapshot
        start_http_server(metric_port)  # expose /metrics
        self.queue = queue.Queue()
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def enqueue(self, q_cmd, q_meas, T_des, config_tag):
        self.queue.put((q_cmd, q_meas, T_des, config_tag))

    def _worker(self):
        while True:
            q_cmd, q_meas, T_des, config_tag = self.queue.get()
            t0 = time.time()
            # FK and Jacobian using Pinocchio APIs
            pin.forwardKinematics(self.model, self.data, q_meas)
            pin.updateFramePlacements(self.model, self.data)
            T_actual = self.data.oMf[self.model.getFrameId('ee_frame')]
            J = pin.computeFrameJacobian(self.model, self.data, q_meas, self.model.getFrameId('ee_frame'))
            # singular values and residual
            s = np.linalg.svd(J, compute_uv=False)
            sigma_min = float(np.min(s))
            # compute pose error in se3 (6-vector)
            err6 = pin.log6(pin.se3Inv(T_des).dot(T_actual))
            residual = float(np.linalg.norm(err6))
            # update Prometheus metrics
            GAUGE_RESIDUAL.set(residual)
            GAUGE_SIGMA_MIN.set(sigma_min)
            GAUGE_COMPUTE_MS.set((time.time()-t0)*1000.0)
            # structured log payload
            payload = {
               'timestamp': time.time(),
               'config': config_tag,
               'q_cmd': q_cmd.tolist(),
               'q_meas': q_meas.tolist(),
               'residual': residual,
               'sigma_min': sigma_min
            }
            # publish as JSON string (non-blocking ROS2 pub)
            self.pub.publish(String(data=json.dumps(payload)))
            self.queue.task_done()