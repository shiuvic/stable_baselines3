import pybullet as p
import pybullet_data
import os
import time

class Plane:
    def __init__(self, client):
        f_name = os.path.join(os.path.dirname(__file__), 'simpleplane.urdf')
        f_name2 = os.path.join(os.path.dirname(__file__), 'red_.urdf')
        f_name3 = os.path.join(os.path.dirname(__file__), 'red_.urdf')
        f_name4 = os.path.join(os.path.dirname(__file__), 'red_.urdf')
        goal = os.path.join(os.path.dirname(__file__), 'goal.urdf')
        p.loadURDF(fileName=f_name,basePosition=[0, 0, 0],physicsClientId=client)
        self.goal = p.loadURDF(fileName=goal,basePosition=[4, 0, 0],physicsClientId=client)
        self.red_1 = p.loadURDF(f_name2, [1.5, -0.8, 0], physicsClientId=client)
        self.red_2 = p.loadURDF(f_name3, [2.5, -0.2, 0], physicsClientId=client)
        self.red_3 = p.loadURDF(f_name4, [0.4, 0.2, 0], physicsClientId=client)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.loadURDF("plane.urdf")

    def pos_return(self,num):
        if num == 1:
            return (p.getBasePositionAndOrientation(self.red_1)[0])
        elif num == 2:
            return (p.getBasePositionAndOrientation(self.red_2)[0])
        elif num == 3:
            return (p.getBasePositionAndOrientation(self.red_3)[0])
        elif num == 0:
            return (p.getBasePositionAndOrientation(self.goal)[0])
