# Reference used for simulation: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

from re import L
import pybullet
import time
import pybullet_data
import random
import math
import vector

tick = 0

physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0,0,-9.81)
pybullet.planeId = pybullet.loadURDF("plane.urdf")

# Goal velocities of ramp
rotations = [30,35,40,45,50]

for rotation in rotations:
	# Different motion
	simulationStart = time.time();
	for i in range(5):
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

		cubeId = pybullet.loadURDF("physics_block.urdf", (-5, 0, 10), inclineOrientation)
		


		inclineTransform = (0,0,0)
		quatrotation = pybullet.getQuaternionFromEuler((0,rotation,0));

		# Actually simulate
		while continueSimulation:
			deltaTime = time.time() - deltaTime

			cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(cubeId)

			if simulationStep >= 500 and not forceApplied:
				startPos = cubePos
				pybullet.applyExternalForce(cubeId, -1, cubePos, (1,0,0), pybullet.WORLD_FRAME)
				forceApplied = True
			
			if math.sqrt(cubePos[0]^2 + cubePos[1]^2 + cubePos[2]^2):
				continueSimulation = False

			simulationTime += 1./240.
			time.sleep(1./240.)
			pybullet.stepSimulation()
		pybullet.removeBody(cubeId)
		pybullet.removeBody(inclinePlaneId)
		print("Finished simulation for 1")

pybullet.disconnect()