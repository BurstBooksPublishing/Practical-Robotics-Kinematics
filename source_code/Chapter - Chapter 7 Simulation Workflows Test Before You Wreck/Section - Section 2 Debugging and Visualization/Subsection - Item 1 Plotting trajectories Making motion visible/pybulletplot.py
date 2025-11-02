import pybullet as p, pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# init sim (DIRECT or GUI)
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot = p.loadURDF("ur5.urdf", useFixedBase=True)  # URDF in pybullet_data

link_index = 7  # UR5 end-effector link index (verify for your URDF)
q0 = np.array([0, -1.0, 0, -1.5, 0, 0])            # start joint config
qf = np.array([0.5, -0.5, 0.2, -1.2, 0.3, 0.1])    # goal joint config

T = 3.0                # motion duration (s)
dt = 0.02              # sampling period (s)
times = np.arange(0, T+dt, dt)
positions = []

def s(t, T):           # cubic time scaling (Eq. 1)
    tau = t / T
    return 3*tau**2 - 2*tau**3

for t in times:
    qt = q0 + s(t, T)*(qf-q0)
    # set joint states without dynamics (kinematic FK)
    for i, qi in enumerate(qt):
        p.resetJointState(robot, i, qi)
    # FK: getLinkState returns tuple, position at index 4
    link_state = p.getLinkState(robot, link_index, computeForwardKinematics=True)
    pos = np.array(link_state[4])
    positions.append(pos)

positions = np.vstack(positions)
vels = np.vstack([np.zeros(3), np.diff(positions, axis=0)/dt])  # numeric velocity

# plotting
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:,0], positions[:,1], positions[:,2], '-',
        color='C0', lw=2)                     # trace
ax.scatter(positions[0,0],positions[0,1],positions[0,2], color='green') # start
ax.scatter(positions[-1,0],positions[-1,1],positions[-1,2], color='red') # goal
# quiver for velocity every N samples
N = max(1, int(0.1/dt))
ax.quiver(positions[::N,0], positions[::N,1], positions[::N,2],
          vels[::N,0], vels[::N,1], vels[::N,2], length=0.05, normalize=True)
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
plt.show()
p.disconnect()