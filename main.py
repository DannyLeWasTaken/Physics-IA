# Reference used for simulation: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

from re import L
import pybullet
import time
import pybullet_data

tick = 0

physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0,0,-9.81)
pybullet.planeId = pybullet.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = pybullet.getQuaternionFromEuler([0,0,0])
boxId = pybullet.loadURDF("physics_block.urdf", startPos, startOrientation)

for velocity in range(5):
	# Different motion
	simulationStart = time.time();
	for repeat in range(5):
		# Set up simulation
		deltaTime = time.time()

		boxId = pybullet.loadURDF("physics_block.urdf", startPos, startOrientation)

		# Actually simulate
		for step in range(1000):
			deltaTime = time.time() - deltaTime
			time.sleep(1./100.)


for i in range(10000):
	pybullet.stepSimulation()
	time.sleep(1./100.)

cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(boxId)
print(cubePos, cubeOrn)
pybullet.disconnect()