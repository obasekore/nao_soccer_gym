import gym
import qi
import humanoid_gym
import pybullet as p
import numpy as np
import time

env = gym.make("nao-v1")

s = qi.Session()
IP = "127.0.0.1"
port = 9559
# env.joint_names
# https://developer.softbankrobotics.com/sites/default/files/repository/70_html_nao/_images/naov6_almotion_names.png
s.connect("tcp://{}:{}".format(IP,port))

motion = s.service("ALMotion")

# name            = "LLeg" #RLeg
# frame           = motion.FRAME_WORLD
# useSensorValues = True
# result          = motion_service.getPosition(name, frame, useSensorValues)

p.loadURDF("goal_post.urdf", globalScaling=1)

def swapAction(action, lstPos1, lstPos2):
    """
    #     lstPos1 = [start, length]
    #     lstPos2 = [start, length]
    """
    start1, start2 = lstPos1[0], lstPos2[0]
    end1, end2 = lstPos1[0] + lstPos1[1], lstPos2[0] + lstPos2[1]
    
    action[start2:end2], action[start1:end1] = action[start1:end1], action[start2:end2]
    
    return action

# Ball config	
#68–70 cm
mass = 0.044 #410 - 450 g
z = 0.314
root = [0, 0, 0]
sphereRadius = 0.05 #0.05
legPose = np.array(env.robot.getPosition())+np.array([0.16,-0.08,0])
x, y, z = legPose #result #0, 0, 0



basePosition = [x, y, z] # location of the leg
baseOrientation = [0, 0, 0, 1]
colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
visualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius)

sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId, basePosition, baseOrientation)


for i in range(100_000):	
    actions = np.array(motion.getAngles("Body",False)) #env.action_space.sample()
    lstPos1 = [2,6]		#LArm
    lstPos2 = [8,6]		#RLeg
    corrected_actions = np.array(swapAction(actions.tolist().copy(), lstPos1, lstPos2))

    lstPos3 = [14,6]		#LLeg
    corrected_actions = np.array(swapAction(corrected_actions.tolist().copy(), lstPos2, lstPos3))

    env.step(corrected_actions)
	


    # p.applyExternalForce(sphereUid,-1, force, [0,0,0], p.LINK_FRAME)

    # p.stepSimulation()

# time.sleep(2)
input('Press enter to continue....')

env.close()
s.close()