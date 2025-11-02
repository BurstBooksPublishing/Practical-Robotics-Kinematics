#include 
#include 
#include 

void planAndExecute(moveit::planning_interface::MoveGroupInterface &move_group)
{
  // add collision box in front of robot (PlanningSceneInterface API)
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::CollisionObject box;
  box.id = "fixture_box";
  box.header.frame_id = move_group.getPlanningFrame();
  // define primitive and pose (omitted detailed construction for brevity)
  // psi.applyCollisionObject(box); // apply to planning scene

  move_group.setPoseReferenceFrame("base_link");               // set frame
  move_group.setMaxVelocityScalingFactor(0.5);                 // safety scaling
  geometry_msgs::Pose target_pose = /* measured target pose */;

  move_group.setPoseTarget(target_pose);                       // request IK + plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!ok) { /* handle planning failure: relax constraints or retry */ }

  // smooth time parameterization before execution
  robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), plan.trajectory_);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool worked = iptp.computeTimeStamps(rt);                    // respects v & a limits
  if(worked) {
    rt.getRobotTrajectoryMsg(plan.trajectory_);
    move_group.execute(plan);                                  // uses controller action
  } else { /* fallback: execute with conservative scaling or replan */ }
}