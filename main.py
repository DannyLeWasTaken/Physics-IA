# Reference used for simulation: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

from re import L
import pybullet
import time
import pybullet_data
import random
import math
import numpy as np

tick = 0

physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0,0,-9.81)
pybullet.planeId = pybullet.loadURDF("plane.urdf")

# Goal velocities of ramp
rotations = [30,35,40,45,50]

def vectorMangitudes(vec1, vec2):
	# Determine magnitude in 3d space
	return math.sqrt(((vec1[0] - vec2[0])**2) + ((vec1[1] - vec2[1])**2) + ((vec1[2] - vec2[2])**2))

for rotation in rotations:
	# Different motion
	simulationStart = time.time();
	for i in range(1):
		# Set up simulation
		deltaTime = time.time()
		simulationTime = 0
		simulationStep = 0
		forceApplied = False
		continueSimulation = True
		
		inclineOrientation = pybullet.getQuaternionFromEuler((0, rotation * (math.pi/180), 0))

		inclinePlaneId = pybullet.loadURDF("plane.urdf", (0,0,0), inclineOrientation)
		pybullet.setCollisionFilterPair(inclinePlaneId, pybullet.planeId, -1, -1, 0)
		pybullet.changeDynamics(inclinePlaneId, -1, mass=0)

		rayResult = pybullet.rayTest((-5,0,10), (-5,0,0))

		cubeId = pybullet.loadURDF("physics_block.urdf", (-5, 0, 8), inclineOrientation)

		dummyId = pybullet.loadURDF("physics_block.urdf", (2, 1, 0), inclineOrientation)

		startPos = None


		inclineTransform = (0,0,0)
		quatrotation = pybullet.getQuaternionFromEuler((0,rotation,0));

		# Actually simulate
		while continueSimulation:
			deltaTime = time.time() - deltaTime

			cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(cubeId)
			tempPos, tempOrn = pybullet.getBasePositionAndOrientation(dummyId)

			if simulationStep >= 400 and forceApplied == False:
				startPos = cubePos
				pybullet.applyExternalForce(cubeId, -1, cubePos, (10,0,0), pybullet.WORLD_FRAME)
				pybullet.applyExternalForce(dummyId, -1, tempPos, (0,0,10), pybullet.WORLD_FRAME)
				forceApplied = True
				print("Pushed!")
			
			if startPos != None and vectorMangitudes(startPos, cubePos) >= 1:
				continueSimulation = False

			simulationStep += 1
			simulationTime += 1./240.
			time.sleep(1./240.)
			pybullet.stepSimulation()
		pybullet.removeBody(cubeId)
		pybullet.removeBody(inclinePlaneId)
		print("Finished simulation for 1")

pybullet.disconnect()