import numpy as np
import pybullet as p
import pybullet_data
import os
import time


class Plane:
    def __init__(self, client):
        f_name = os.path.join(os.path.dirname(__file__), 'square.urdf')
        goal = os.path.join(os.path.dirname(__file__), 'square_blue.urdf')
        red_goal = os.path.join(os.path.dirname(__file__), 'square_red.urdf')
        p.loadURDF(fileName=f_name,basePosition=[1.2, 0, 3.2],physicsClientId=client)

        self.goal = p.loadURDF(fileName=goal,basePosition=[1.35, -1, 3.21],physicsClientId=client)
        self.red_goal = p.loadURDF(fileName=red_goal, basePosition=[1.35, 1, 3.21], physicsClientId=client)


    def pos_return(self,num):
        if num == 1:
            return (p.getBasePositionAndOrientation(self.goal)[0])
        elif num == 2:
            return (p.getBasePositionAndOrientation(self.red_goal)[0])

    def get_goal_ORE(self):
        _, ore = p.getBasePositionAndOrientation(self.goal)
        return np.array(ore)


if __name__ == '__main__':
    cli = p.connect(p.GUI)
    Plane(cli)
    while True:
        time.sleep(0.1)

