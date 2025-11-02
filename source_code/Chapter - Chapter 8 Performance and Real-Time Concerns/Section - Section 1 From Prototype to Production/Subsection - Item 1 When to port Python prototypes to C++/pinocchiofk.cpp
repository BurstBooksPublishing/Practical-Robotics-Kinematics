#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

pinocchio::Model model;                          // build model from URDF once
pinocchio::urdf::buildModel("robot.urdf", model);
pinocchio::Data data(model);

// q is Eigen::VectorXd joint positions (size model.nq)
pinocchio::forwardKinematics(model, data, q);   // fast FK (C++)
pinocchio::updateFramePlacements(model, data);  // update frames for end-effector poses
auto pose = data.oMf[frame_id].translation();   // use Eigen API for pose math