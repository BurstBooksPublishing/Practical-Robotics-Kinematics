#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <chrono>

// assume model and data are constructed once off-line
pinocchio::Model model;          // loaded at startup
pinocchio::Data data(model);
const int n = model.nq;
Eigen::VectorXd q(n);           // preallocated joint vector
Eigen::Matrix J(6,n); // preallocated jacobian

void control_loop_iteration(const Eigen::VectorXd &q_in) {
  // copy into preallocated buffer (no allocations)
  q = q_in; // Eigen does stack semantics if sizes fixed; fine here

  // timing start
  auto t0 = std::chrono::steady_clock::now();

  // forward kinematics and frame placement (no dynamic alloc)
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // get frame Jacobian into preallocated J
  const pinocchio::FrameIndex ef = /* end-effector frame index */;
  pinocchio::getFrameJacobian(model, data, ef, pinocchio::LOCAL, J);

  // use J for control law, compute torques, etc.

  // timing end
  auto t1 = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  // log elapsed if needed (off the real-time thread)
}