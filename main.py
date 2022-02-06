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

# Goal velocities of ramp
velocities = [0,4,8,16,32]

for velocity in velocities:
	# Different motion
	simulationStart = time.time();
	for i in range(5):
		# Set up simulation
		deltaTime = time.time()
		lastPos = startPos
		cubeId = pybullet.loadURDF("physics_block.urdf", startPos, startOrientation)
		pybullet.applyExternalForce(cubeId, -1, (velocity, 0, 0), startPos, pybullet.WORLD_FRAME);

		# Actually simulate
		for step in range(1000):
			deltaTime = time.time() - deltaTime
			pybullet.stepSimulation()

			cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(cubeId)
			lastPos = cubePos;

			time.sleep(1./240.)
		
		pybullet.removeBody(cubeId)
		print("Finished simulation for 1")

pybullet.disconnect()