import gym
import numpy as np
import math
import pybullet as p
from robot.resources.core.op3 import OP3
from robot.resources.plane import Plane
from robot.resources.walker import Walker
from gym.utils import seeding
from robot.resources.savereward import save
import time

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
        '''

        self.observation_space = gym.spaces.box.Box(
            low=np.zeros((50,50,4),dtype=np.float32),
            high=np.full((50,50,4),255,dtype=np.float32))
        # self.observation_space = gym.spaces.box.Box(
        #     low=np.array([-2, -2, -2, 0.0, 0.0, 0.0], dtype=np.float32),
        #     high=np.array([2, 2, 2, 0.2, 0.2, 0.35], dtype=np.float32))

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
        self.get_point = False
        np_resource = np.dtype([("resource", np.ubyte, 1)])
        # self.reset
        arms2point = np.array([0.16, 0.13, 0.32])
        self.arm2point_len = np.linalg.norm(arms2point)
        self.grab_counter = 0
        self.reward = 0
    def step(self, action):
        self.state += action
        self.OP3.apply_action(self.state)
        # print("State = ",self.state)
        OP3_ob = self.OP3.get_observation()
        self.state = self.OP3.get_state()
        # pos = self.OP3.get_position()
        # self.reward = self.reward_fun(dis)
        ob = OP3_ob
        ob = np.array(ob, dtype=np.float32)
        # print("reward = ", reward)
        info = {}
        return ob, self.reward, self.done, info

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    @property
    def reset(self):
        # 初始化所有東西
        print('reward = ',self.reward)
        # save(self.reward)
        # p.resetSimulation(self.client)
        self.done = False
        self.OP3.reset()
        OP3_ob = self.OP3.get_observation()
        ob = OP3_ob
        self.state = self.OP3.get_state()
        return np.array(ob,dtype=np.float32)

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect(self.client)

    def reward_fun(self, distance):
        t = 10
        r = -distance / self.arm2point_len
        if distance < 0.03 and (not self.done):
            r += 1.
            self.grab_counter += 1
            r += self.grab_counter
            if self.grab_counter > t:
                # r += 10.
                self.done = True
                self.count = 0
        elif distance > 0.03:
            self.grab_counter = 0
            self.done = False

        return r
