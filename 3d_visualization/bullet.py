import time
import numpy as np
import pybullet as p
import pybullet_data as pd
import imageio

Xout = np.loadtxt('Xout.txt')

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

floor = p.loadURDF("plane.urdf") 
robot = p.loadURDF("mini_cheetah/mini_cheetah.urdf", [0, 0, 0.5])

numJoints = p.getNumJoints(robot)
p.changeVisualShape(robot, -1, rgbaColor=[0.8, 0.8, 0.8, 1])
for j in range(numJoints):
	p.changeVisualShape(robot, j, rgbaColor=[0.8, 0.8, 0.8, 1])

dt = 1./20. # Should be 1./100. Illustration of slow motion only
p.setTimeStep(dt)

ims = []

while True:
	for counter in range(len(Xout)):
		state = Xout[counter]

		p.resetBasePositionAndOrientation(robot, [state[0], 0, state[1]], p.getQuaternionFromEuler([0, -state[2], 0]))

		p.resetJointState(robot, 1, state[3])
		p.resetJointState(robot, 2, state[4])
		p.resetJointState(robot, 5, state[3])
		p.resetJointState(robot, 6, state[4])

		p.resetJointState(robot, 9, state[5])
		p.resetJointState(robot, 10, state[6])
		p.resetJointState(robot, 13, state[5])
		p.resetJointState(robot, 14, state[6])

		#print(p.getDebugVisualizerCamera())
		_, _, rgbPixels, _, _ = p.getCameraImage(1024, 711, (0.6427874565124512, -0.4393850862979889, 0.6275069713592529, 0.0, 0.766044557094574, 0.3686877191066742, -0.5265406966209412, 0.0, -0.0, 0.8191520571708679, 0.5735763907432556, 0.0, 5.960464477539063e-08, -0.0, -1.6000030040740967, 1.0), (0.6943359375, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0))
		ims.append(rgbPixels)

		time.sleep(dt)
	break

imageio.mimwrite("video.avi", ims)
