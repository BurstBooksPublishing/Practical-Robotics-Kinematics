#include 
#include 
#include 
#include 
#include 

// --- build chain (pseudo-code, fill segment transforms per robot) ---
KDL::Chain chain;
// for each wheel i: add segment with revolute joint about wheel axis
// chain.addSegment(KDL::Segment("wheel_i", KDL::Joint(KDL::Joint::RotAxis, axis), frame_to_wheel));
// ...

int n = chain.getNrOfJoints();
KDL::JntArray q(n); // wheel angles (not used for stateless wheels)
KDL::Jacobian J(n);
KDL::ChainJntToJacSolver jacSolver(chain);

// compute Jacobian once per control tick
jacSolver.JntToJac(q, J);

// convert KDL::Jacobian to Eigen::MatrixXd (3 x n planar part)
Eigen::MatrixXd Je(3, n);
for (int i=0;i<3;++i) for (int j=0;j
\subsection{Item 3:  Simulation with MuJoCo: Testing before building}
The preceding KDL examples showed how to structure base-control kinematics, and the omni/mecanum discussion exposed the nonintuitive wheel-to-twist mapping you must validate. Simulation with MuJoCo lets you exercise those mappings, tune contact and actuator parameters, and catch control assumptions before building hardware.

Concept — why MuJoCo for mobile bases
\begin{itemize}
\item MuJoCo models continuous contact dynamics, compliant contacts, and solver parameters that matter for wheel-ground interaction.  
\item Use the official Python binding (package name \lstinline|mujoco|) for current APIs and performance. Older projects may still use \lstinline|mujoco_py|; prefer the official API for new work.  
\item Typical uses: validate wheel Jacobians, tune friction and actuator gains, verify slip and tipping conditions, and run long batch tests for controller robustness.
\end{itemize}

Theory — wheel-to-platform mapping and what to test
\begin{itemize}
\item Represent the wheel-to-platform kinematic mapping as a linear map from wheel speeds to body twist:
\begin{equation}[H]\label{eq:wheel_jac}
\mathbf{v} = J\,\boldsymbol{\omega},
\end{equation}
where $\mathbf{v}=[v_x,\;v_y,\;\omega_z]^\top$ is the planar body twist, and $\boldsymbol{\omega}$ stacks wheel angular rates.  
\item For a common four-wheel mecanum layout with roller angle $\alpha=\pi/4$ and wheel radius $r$, a compact Jacobian is:
\begin{equation}[H]\label{eq:mecanum_J}
J = \frac{r}{4}
\begin{bmatrix}
1 & 1 & 1 & 1\\[4pt]
-1 & 1 & 1 & -1\\[4pt]
-\frac{1}{L} & \frac{1}{L} & -\frac{1}{L} & \frac{1}{L}
\end{bmatrix},
\end{equation}
with $L$ the effective half-length plus half-width around the center of rotation. Validate (\ref{eq:wheel_jac}) numerically in simulation.
\end{itemize}

Example — model, command, and validate in MuJoCo
\begin{itemize}
\item Create an MJCF or converted URDF model with:
\begin{enumerate}
\item a planar-freebody base (floating joint for 3D) or a planar constraint for 2D tests,
\item revolute joints for each wheel with rollers modeled or approximated,
\item actuators of type \texttt{velocity} or \texttt{motor} so you can command angular rates directly.
\end{enumerate}
\item The test steps:
\begin{enumerate}
\item command a set of wheel rates $\boldsymbol{\omega}$ via \lstinline|sim.data.ctrl|,
\item step the sim for $T$ seconds at timestep $\Delta t$,
\item compute measured twist $\hat{\mathbf{v}}$ by finite differences of base pose,
\item compare $\hat{\mathbf{v}}$ with $J\,\boldsymbol{\omega}$.
\end{enumerate}
\end{itemize}

Code listing: minimal MuJoCo test harness
\begin{lstlisting}[language=Python,caption={MuJoCo test to command wheel speeds and validate base twist.},label={lst:mujo_sim}]
import mujoco      # official binding
import numpy as np
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("mecanum_base.xml")  # MJCF path
sim = mujoco.MjSim(model)                                # create sim
# indices: assume actuators map to wheels in order
act_inds = range(model.nu)                               # actuator count
dt = model.opt.timestep
def base_pose(sim):                                       # read base pose (x,y,quat)
    return np.copy(sim.data.qpos[:7])

# command wheel rates and run
omega_cmd = np.array([10., 10., 10., 10.])               # rad/s example
sim.data.ctrl[:] = omega_cmd                             # set velocity-targets
pose0 = base_pose(sim)
for _ in range(int(0.5/dt)):                             # simulate 0.5s
    sim.step()
pose1 = base_pose(sim)
# finite-difference (planar): vx,vy,omega_z
dx = (pose1[0]-pose0[0])/(0.5)                           # x displacement / T
dy = (pose1[1]-pose0[1])/(0.5)                           # y displacement / T
# quaternion to yaw (z) finite diff
q0 = pose0[3:7]; q1 = pose1[3:7]
# convert quaternions to yaw using mujoco functions or simple math (placeholder)
yaw0 = mujoco.mju_quat2euler(q0)[2]; yaw1 = mujoco.mju_quat2euler(q1)[2]
omega_z = (yaw1 - yaw0)/(0.5)
measured_v = np.array([dx, dy, omega_z])
# expected via analytic J (from code or parameters)
r = 0.05; L = 0.3
J = (r/4)*np.array([[1,1,1,1],[-1,1,1,-1],[-1/L,1/L,-1/L,1/L]])
expected_v = J.dot(omega_cmd)
print("meas:", measured_v, "exp:", expected_v)
viewer.launch(sim)                                       # optional GUI