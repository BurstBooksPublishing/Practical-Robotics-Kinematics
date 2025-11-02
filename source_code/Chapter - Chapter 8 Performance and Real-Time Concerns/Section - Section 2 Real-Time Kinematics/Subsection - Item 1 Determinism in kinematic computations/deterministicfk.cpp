#include 
#include 
#include 
#include 
#include 
#include 

void realtime_loop(pinocchio::Model &model, pinocchio::Data &data, 
                   const std::vector &frames)
{
  // lock memory to avoid page faults (call once at init)
  mlockall(MCL_CURRENT | MCL_FUTURE);
  // fix FP rounding mode for repeatability
  fesetround(FE_TONEAREST);

  // fixed-size Eigen vector for joint state (avoid dynamic allocation)
  Eigen::Matrix q(model.nq); // allocate once
  Eigen::Matrix task_pose;               // fixed-size

  // Main real-time loop (no allocations here)
  while (true) {
    // read sensors into preallocated q buffer (RT-safe)
    read_joint_positions_rt(q.data(), model.nq); // user-provided RT-safe read

    // forward kinematics (preallocated data)
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // example: get frame  end-effector placement (no heap)
    const auto &placement = data.oMf[frames[0]];
    task_pose.head<3>() = placement.translation();
    task_pose.tail<3>() = pinocchio::log6(placement.rotation()); // minimal twist

    // compute Jacobian into preallocated matrix
    Eigen::Matrix J(6, model.nv); // allocated before loop
    pinocchio::getFrameJacobian(model, data, frames[0], pinocchio::LOCAL, J);

    // publish or control using RT-safe mechanism
    rt_publish(task_pose.data(), J.data());
    wait_until_next_period(); // RT-timer sleep
  }
}

// In main(): parse model, create Data, pin thread to CPU, and start realtime_loop()
// compile with -fno-fast-math, link single-threaded LAPACK/BLAS if used.