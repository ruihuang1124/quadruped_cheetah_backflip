import time
import numpy as np
import pybullet as p
import pybullet_data as pd

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

		time.sleep(dt)


