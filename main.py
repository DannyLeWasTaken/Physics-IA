# Reference used for simulation: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
# Copper 	Cast Iron 	Clean and Dry 	1.05 	0.29

from re import L
from struct import unpack
import pybullet
import time
import pybullet_data
import random
import math
import csv
import os
import numpy as np

# DEFINE CONSTANTS
#https://stackoverflow.com/questions/4060221/how-to-reliably-open-a-file-in-the-same-directory-as-the-currently-running-scrip
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))


# CONFIGRUATION
numberTrials = 5
rotations = [20,25,30,35,40,45,50,55,60,80]

recordedData = {}

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
	newMin = min + random.uniform(-(min * actualPercent), (min * actualPercent)) 
	newMax = max + random.uniform(-(max * actualPercent), (max * actualPercent)) 
	return random.uniform(newMin, newMax) / decimals

def generateInaccuracy(num, decimals=1, deviate=1, percentunforseen=10):
	return generateRIU(num - deviate, num + deviate, decimals, percentunforseen)

def generatePercentInaccuracy(num, decimals=1, percentdeviate=10, percentunforseen=10):
	actualPercent = percentdeviate/100
	return generateRIU(num - (num * actualPercent), num + (num * actualPercent), decimals, percentunforseen)

physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0,0,-9.81)
pybullet.planeId = pybullet.loadURDF("plane.urdf")

for rotation in rotations:
	recordedData[rotation] = []
	# Different motion
	for i in range(numberTrials):
		# Set up simulation
		simulationStart = time.time();
		simulationEnd = 0;
		simulationTime = 0
		simulationStep = 0
		continueSimulation = True
		startPos = None

		maxDistanceCanTravel = 1
		inclineOrientation = pybullet.getQuaternionFromEuler((0, getDegreesFromRadians( generateInaccuracy(rotation, 1, 2, 10) ), 0))

		inclinePlaneId = pybullet.loadURDF("plane.urdf", (0,0,0), inclineOrientation)
		pybullet.setCollisionFilterPair(inclinePlaneId, pybullet.planeId, -1, -1, 0)
		pybullet.changeDynamics(inclinePlaneId, -1, mass=0)

		rayResult = pybullet.rayTest((-5,0,100), (-5,0,0))
		cubeStartPos = rayResult[0][3]
		cubeStartPos = (cubeStartPos[0], cubeStartPos[1] + 1, cubeStartPos[2])

		cubeId = pybullet.loadURDF("physics_block.urdf", cubeStartPos, inclineOrientation)


		inclineTransform = (0,0,0)
		quatrotation = pybullet.getQuaternionFromEuler((0,rotation,0));

		# Actually simulate
		while continueSimulation:

			cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(cubeId)
			
			# Check if there is something underneath
			rayResult = pybullet.rayTest(cubePos, (cubePos[0], cubePos[0] - 0.1, cubePos[0]))
			if rayResult[0][0] != None and startPos == None:
				startPos = cubePos
				# Generate unforseen random force generated by a simulated hand
				pybullet.applyExternalForce(cubeId, -1, cubePos, ( generateInaccuracy(0, 100, 0.2, 10), generateInaccuracy(0, 100, 0.2, 10), generateInaccuracy(0, 100, 0.2, 10) ), pybullet.WORLD_FRAME)
				# https://stackoverflow.com/questions/49639163/move-an-object-towards-a-target-in-pybullet

				#forceTowardsPoint = 300 * (np.array(cubePos) - np.array((0,0,0)))
				#pybullet.applyExternalForce(cubeId, -1, cubePos, forceTowardsPoint, pybullet.WORLD_FRAME)
			
			if (time.time() - simulationStart)>=10:
				continueSimulation = False
				simulationEnd = 0
			elif (startPos != None and vectorMangitudes(startPos, cubePos) >= maxDistanceCanTravel):
				continueSimulation = False
				simulationEnd = time.time()

			simulationStep += 1
			simulationTime += 1./240.
			time.sleep(1./240.)
			pybullet.stepSimulation()
		# Simulation stop, clean up!
		pybullet.removeBody(cubeId)
		pybullet.removeBody(inclinePlaneId)
		
		if simulationEnd != 0:
			print("===TRIAL REPORT===\nSimulation Trial: {trial}.\nAngle: {angle}, Delta T: {dt}\n===END REPORT===".format(trial = i + 1, angle = rotation, dt = simulationEnd - simulationStart))
			recordedData[rotation].append(simulationEnd - simulationStart)
		else:
			recordedData[rotation].append("N/A")

		print("===TRIAL REPORT===\nSimulation Trial: {trial}.\nAngle: {angle}, Delta T: {dt}\n===END REPORT===".format(trial = i + 1, angle = rotation, dt = (simulationEnd != 0 and simulationEnd - simulationStart) or "N/A" ))


# https://stackoverflow.com/questions/42486764/python-creating-a-new-file-folder-in-the-same-directory-as-the-script
resultsDirectory = os.path.join(__location__, 'results')
roundedSimulation = round(simulationEnd)
csvDirectory = r'{base}/{timeStamp}.csv'.format(base=resultsDirectory, timeStamp=roundedSimulation)

if not os.path.exists(resultsDirectory):
	os.mkdir(resultsDirectory)

if not os.path.exists(csvDirectory):
	open("results\{}.csv".format(roundedSimulation), "x")

with open("results\{}.csv".format(roundedSimulation), 'w') as csvfile:
	writer = csv.writer(csvfile)
	fields = ["Rotation"]
	for x in range(numberTrials):
		fields.append("Trial {number}".format(number=x + 1))
	writer.writerow(fields)

	for key in rotations:
		row = recordedData[key]
		row.insert(0, str(key))
		writer.writerow(row)
	



pybullet.disconnect()