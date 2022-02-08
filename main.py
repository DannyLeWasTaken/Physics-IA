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

# Goal velocities of ramp
rotations = [30,35,40,45,50]

def vectorMangitudes(vec1, vec2):
	# Determine magnitude in 3d space
	return math.sqrt(((vec1[0] - vec2[0])**2) + ((vec1[1] - vec2[1])**2) + ((vec1[2] - vec2[2])**2))

def getDegreesFromRadians(rad):
	return rad * (math.pi/180)

def generateRIU(min, max, decimals=1, unforseenpercent=10):
	#generateRandomIntegerUnforseen
	# generates a random number including an unforseen randomness to prevent predictability
	actualPercent = unforseenpercent / 100
	min *= decimals;
	max *= decimals;
	newMin = min + random.randint(-(min * actualPercent), (min * actualPercent)) 
	newMax = max + random.randint(-(max * actualPercent), (max * actualPercent)) 
	return random.randint(newMin, newMax) / decimals

def generateInaccuracy(num, decimals=1, deviate=1, percentunforseen=10):
	return generateRIU(num - deviate, num + deviate, decimals, percentunforseen)

def generatePercentInaccuracy(num, decimals=1, percentdeviate=10, percentunforseen=10):
	actualPercent = percentdeviate/100
	return generateRIU(num - (num * actualPercent), num + (num * actualPercent), decimals, percentunforseen)

for rotation in rotations:
	# Different motion
	for i in range(5):
		# Set up simulation
		simulationStart = time.time();
		simulationEnd = 0;
		simulationTime = 0
		simulationStep = 0
		continueSimulation = True
		maxDistanceCanTravel = 1 + (random.randint(-300, 300)/1000)
		
		inclineOrientation = pybullet.getQuaternionFromEuler((0, getDegreesFromRadians(rotation), 0))

		inclinePlaneId = pybullet.loadURDF("plane.urdf", (0,0,0), inclineOrientation)
		pybullet.setCollisionFilterPair(inclinePlaneId, pybullet.planeId, -1, -1, 0)
		pybullet.changeDynamics(inclinePlaneId, -1, mass=0)

		rayResult = pybullet.rayTest((-5,0,10), (-5,0,0))
		cubeStartPos = rayResult[0][3]
		cubeStartPos = (cubeStartPos[0], cubeStartPos[1] + 1, cubeStartPos[2])

		cubeId = pybullet.loadURDF("physics_block.urdf", cubeStartPos, inclineOrientation)

		startPos = None


		inclineTransform = (0,0,0)
		quatrotation = pybullet.getQuaternionFromEuler((0,rotation,0));

		# Actually simulate
		while continueSimulation:

			cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(cubeId)
			
			# Check if there is something underneath
			rayResult = pybullet.rayTest(cubePos, (cubePos[0], cubePos[0] - 0.1, cubePos[0]))
			if rayResult[0][0] != None and startPos == None:
				startPos = cubePos
			
			if startPos != None and vectorMangitudes(startPos, cubePos) >= maxDistanceCanTravel:
				continueSimulation = False
				simulationEnd = time.time()

			simulationStep += 1
			simulationTime += 1./240.
			time.sleep(1./240.)
			pybullet.stepSimulation()
		# Simulation stop, clean up!
		pybullet.removeBody(cubeId)
		pybullet.removeBody(inclinePlaneId)

		print("===SIMULATION COMPLETED===\nSimulation Trial: {trial}.\nAngle: {angle}, Delta T: {dt}".format(trial = i, angle = rotation, dt = simulationEnd - simulationStart))


pybullet.disconnect()