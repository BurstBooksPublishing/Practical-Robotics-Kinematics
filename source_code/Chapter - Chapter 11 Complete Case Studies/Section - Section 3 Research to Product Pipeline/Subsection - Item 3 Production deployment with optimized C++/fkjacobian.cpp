#include 
#include 
#include 
#include 

class KinematicsModule {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // aligned storage for Eigen types

  KinematicsModule(const std::string &urdf_path, int frame_id)
  : frameId(frame_id)
  {
    pinocchio::urdf::buildModel(urdf_path, model); // one-time parse
    data = pinocchio::Data(model);                  // per-instance data
    q_reserve = Eigen::VectorXd::Zero(model.nq);    // preallocated vector
    jacobian_output = Eigen::MatrixXd::Zero(6, model.nq);
  }

  // Real-time safe call: map input joint array, write into preallocated outputs.
  void compute(const double *q_in, Eigen::Isometry3d &pose_out, Eigen::MatrixXd &J_out) {
    Eigen::Map q_map(q_in, model.nq); // no copy
    q_reserve.noalias() = q_map;            // deterministic copy into aligned buffer

    pinocchio::forwardKinematics(model, data, q_reserve);         // in-place on data
    pinocchio::updateFramePlacements(model, data);                // update placements
    pose_out = pinocchio::getFramePlacement(model, data, frameId);

    pinocchio::computeJointJacobians(model, data, q_reserve);     // updates data.J
    pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL, jacobian_output);
    J_out.noalias() = jacobian_output;                            // copy to user buffer
  }

private:
  pinocchio::Model model;
  pinocchio::Data data;
  int frameId;
  Eigen::VectorXd q_reserve;
  Eigen::MatrixXd jacobian_output;
};