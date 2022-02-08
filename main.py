# Reference used for simulation: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

from re import L
import pybullet
import time
import pybullet_data
import random
import math

tick = 0

physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0,0,-9.81)
pybullet.planeId = pybullet.loadURDF("plane.urdf")


startPos = [0,0,5]
startOrientation = pybullet.getQuaternionFromEuler([0,0,0])

# Goal velocities of ramp
rotations = [30,35,40,45,50]

for rotation in rotations:
	# Different motion
	simulationStart = time.time();
	for i in range(5):
		# Set up simulation
		deltaTime = time.time()
		lastPos = startPos
		
		inclineOrientation = pybullet.getQuaternionFromEuler((0, rotation * (math.pi/180), 0))

		inclinePlaneId = pybullet.loadURDF("plane.urdf", (0,0,0), inclineOrientation)
		pybullet.setCollisionFilterPair(inclinePlaneId, pybullet.planeId, -1, -1, 0)
		pybullet.changeDynamics(inclinePlaneId, -1, mass=0)
		
		cubeId = pybullet.loadURDF("physics_block.urdf", (0,0,0), inclineOrientation)
		


		inclineTransform = (0,0,0)
		quatrotation = pybullet.getQuaternionFromEuler((0,rotation,0));

		# Actually simulate
		for step in range(500):
			pybullet.stepSimulation()
			deltaTime = time.time() - deltaTime

			cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(cubeId)
			


			lastPos = cubePos;

			time.sleep(1./240.)
		
		pybullet.removeBody(cubeId)
		pybullet.removeBody(inclinePlaneId)
		print("Finished simulation for 1")

pybullet.disconnect()