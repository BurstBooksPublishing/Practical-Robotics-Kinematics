#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
// Prebuilt model and data; allocate once before real-time start.
pinocchio::Model model;                 // load once
pinocchio::Data data(model);            // preallocated data
Eigen::VectorXd q(model.nq);           // fixed-size if possible
Eigen::VectorXd qdot(model.nv);

// real-time loop (called at fixed period T_s)
void kinematics_cycle(const Eigen::VectorXd &q_in) {
  // copy without allocating; q_in may be an Eigen Map
  q = q_in;                             // no heap alloc if sizes fixed
  // deterministic forward kinematics; no dynamic allocation
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);   // compute frame poses
  // read end-effector pose from data.oMf[frame_id]
  // perform Jacobian if needed (reuse buffers)
  // ensure no try/catch and no new/delete here in RT path
}