#include <cmath>
#include <Eigen/Dense>
// Compute wheel velocities given robot-frame path points (preallocated),
// commanded linear speed, wheelbase, and pure-pursuit params.
// pts: pointer to first point; npts: number of points
// each point is Eigen::Vector2d (x,y) in robot frame. No heap allocation.
void purePursuitStep(const Eigen::Vector2d* pts, int npts,
                     double v_cmd, double b, double Ld_min,
                     double k_theta, double &v_l, double &v_r)
{
  // 1) find lookahead point: first with distance >= Ld_min
  int idx = -1;
  for (int i = 0; i < npts; ++i) {
    double d2 = pts[i].squaredNorm(); // x^2 + y^2
    if (d2 >= Ld_min*Ld_min) { idx = i; break; }
  }
  if (idx < 0) { // no forward point found: stop or track final point
    v_l = v_r = 0.0;
    return;
  }
  const Eigen::Vector2d &p = pts[idx];
  double Ld = std::sqrt(p.x()*p.x() + p.y()*p.y());
  double alpha = std::atan2(p.y(), p.x());               // lookahead bearing
  double kappa = 2.0 * std::sin(alpha) / std::max(Ld, 1e-6); // curvature (eq. \ref{eq:curvature})
  double omega = v_cmd * kappa;
  // approximate path tangent heading: if next point available, use it for theta_err
  double theta_err = 0.0;
  if (idx+1 < npts) {
    Eigen::Vector2d t = (pts[idx+1] - pts[idx]).normalized();
    double path_yaw = std::atan2(t.y(), t.x());
    theta_err = path_yaw - 0.0; // robot yaw in robot-frame is 0
    // wrap to [-pi,pi]
    if (theta_err > M_PI) theta_err -= 2*M_PI;
    if (theta_err < -M_PI) theta_err += 2*M_PI;
  }
  omega += k_theta * theta_err; // eq. \ref{eq:omega_fb}
  // map to wheel speeds (eq. \ref{eq:wheels})
  v_r = v_cmd + 0.5 * omega * b;
  v_l = v_cmd - 0.5 * omega * b;
}