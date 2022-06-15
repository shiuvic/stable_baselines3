import random

import gym
import numpy as np
import pybullet as p
from robot.resources.core.op3 import OP3
from robot.resources.plane import Plane
from robot.resources.walker import Walker
from gym.utils import seeding
from robot.resources.savereward import save

class Op3Env(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(Op3Env, self).__init__()

        '''
        # 動作空間
        # 第一個維度:['l_sho_pitch'] = [1.5,-1.5]
        # 第二個維度:['l_sho_roll'] = [1.5,-1.5]
        # 第二個維度:['l_el'] = [1.5,-1.5]
        '''

        self.action_space = gym.spaces.box.Box(
            low=np.array([-2.0], dtype=np.float32),
            high=np.array([2.0], dtype=np.float32))

        '''
        # 觀察空間
        # 索引[0,1,2,3] = R,G,B,A = [0,255]
        R,G = [1,50,50]
        '''
        self.observation_space = gym.spaces.box.Box(
            low=np.append(np.zeros((50, 50, 4)).flatten(), np.full((50, 2), [0, 0]).flatten()),
            high=np.append(np.full((50, 50, 4),255).flatten(), np.full((50, 2), [5, 5]).flatten()))

        self.np_random, _ = gym.utils.seeding.np_random()

        # 選擇連結方式
        # self.client = p.connect(p.DIRECT)
        self.client = p.connect(p.GUI)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI)
        # 加速訓練
        p.setTimeStep(1/180, self.client)
        # 初始化所有東西
        self.OP3 = Walker(self.client)
        self.plane = Plane(self.client)
        self.done = False

        self.reward = 0

        self.red_goal = self.plane.pos_return(1)
        self.blue_goal = self.plane.pos_return(2)

        p.resetDebugVisualizerCamera(5.0, 270, -89, [1, 0, 0])

    def step(self, action):
        self.state += action
        self.OP3.apply_action(self.state)
        # print("State = ",self.state)
        OP3_ob = self.OP3.get_observation()
        self.state = self.OP3.get_state()
        self.reward = self.reward_fun()
        # save(self.reward)
        info = {}
        return OP3_ob, self.reward, self.done, info

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    @property
    def reset(self):
        # 初始化所有東西
        self.OP3.reset()
        self.done = False
        OP3_ob = self.OP3.get_observation()
        self.state = self.OP3.get_state()
        return np.array(OP3_ob, dtype=np.float32)

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect(self.client)

    def reward_fun(self):
        robot = self.OP3.get_position() - [0, 0, 0.15]
        x = robot[0]
        y = abs(robot[1])
        red_pos = abs(np.linalg.norm(np.asarray(self.red_goal) - np.asarray(robot)))
        goal_pos = abs(np.linalg.norm(np.asarray(self.blue_goal) - np.asarray(robot)))
        r = (-goal_pos / 2)*20
        if red_pos < 0.4:
            r -= 10
            self.done = True
        elif -0.3 > x or y > 2 or x > 2.7:
            r -= 5
        elif goal_pos < 0.4:
            r += 20
            self.done = True

        if self.OP3.is_fallen():
            r -= 5
            self.done = True
        return r
