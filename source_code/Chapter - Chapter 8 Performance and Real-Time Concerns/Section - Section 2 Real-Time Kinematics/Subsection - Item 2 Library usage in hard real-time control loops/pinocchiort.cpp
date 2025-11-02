#include 
#include 
#include 
#include 
#include 
#include 
#include 
#include 

// Preinit and RT-safe main thread; small comments inline.
void *rt_thread(void* arg) {
  // Lock memory to prevent page faults.
  mlockall(MCL_CURRENT | MCL_FUTURE);

  // Construct model and data once.
  pinocchio::Model model;
  pinocchio::urdf::buildModel("robot.urdf", model); // init-time parse
  pinocchio::Data data(model);

  const int nq = model.nq;
  Eigen::VectorXd q(nq); // reuse; avoid realloc inside loop
  Eigen::Vector3d ee_pos; // small fixed-size type -> no heap

  // Pre-warm any caches or lazy inits (call once).
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacement(model, data, model.frames.size()-1);

  // Real-time periodic loop
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  const long period_ns = 1000000L; // 1 ms control period
  while (/* safe shutdown flag checked lock-free */) {
    // sleep until next period
    next.tv_nsec += period_ns;
    if (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    // Read sensors into preallocated buffers (not shown).
    // Compute kinematics using preallocated model/data.
    pinocchio::forwardKinematics(model, data, q);              // deterministic
    pinocchio::updateFramePlacement(model, data, /*frame_id=*/model.frames.size()-1);
    pinocchio::computeJointJacobians(model, data, q);         // deterministic
    // Extract ee position without allocations.
    ee_pos = data.oMf.back().translation();
    // Compute control law and write to actuators...
  }
  return nullptr;
}

// Thread creation and RT priority setup performed in init (not shown).