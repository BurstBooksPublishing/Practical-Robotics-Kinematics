import pybullet as p
import pybullet_data
import numpy as np

p.connect(p.GUI)                          # GUI for fast visual checks
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("ur5.urdf", useFixedBase=True)

joint_indices = [0,1,2,3,4,5]             # planning joints
ee_link = 7                               # UR5 end-effector link index

# simple domain randomization for robustness testing
def randomize_env():
    p.changeDynamics(plane, -1, lateralFriction=0.6 + 0.2*np.random.randn())

def noisy_sensor_read(q):
    return q + 0.005*np.random.randn(*q.shape)  # small Gaussian noise

for trial in range(50):
    randomize_env()
    target_pos = [0.5, 0.0, 0.2 + 0.05*np.random.randn()]  # randomized goal
    ik_sol = p.calculateInverseKinematics(robot, ee_link, target_pos)
    target_joints = [ik_sol[i] for i in joint_indices]
    # step with position control and sensor noise test
    for _ in range(240):                       # 1 second @ 240Hz
        q = np.array([p.getJointState(robot, i)[0] for i in joint_indices])
        q_noisy = noisy_sensor_read(q)
        p.setJointMotorControlArray(robot, jointIndices=joint_indices,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=target_joints)
        p.stepSimulation()