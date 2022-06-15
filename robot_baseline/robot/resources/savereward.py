import matplotlib
import numpy as np
import matplotlib.pyplot as plt
matplotlib.use('tkagg')
rewardarr = []
# plt.figure(figsize=(6, 4))
#     plt.ylabel('reward')
#     plt.xlabel('total_timesteps')

#[DDPG:1,3]PPO 1


def save(reward):
    # rewardarr.append(reward)
    # plt.plot(rewardarr)
    # plt.draw()
    # plt.pause(1./48.)
    # plt.clf()
    z = reward
    arr = np.load('H:/stable_baseline/robot_baseline/robot/resources/reward.npy')
    arr = np.append(arr, z)
    np.save('H:/stable_baseline/robot_baseline/robot/resources/reward.npy', arr)
    # if z > 15:
    #     save_done(z)


def clear():
    a = []
    np.save('H:/stable_baseline/robot_baseline/robot/resources/finish.npy',a)
    np.save('H:/stable_baseline/robot_baseline/robot/resources/reward.npy', a)

def load():
    data1 = np.load('H:/stable_baseline/robot_baseline/robot/resources/reward.npy')
    # data1 = np.load('H:/stable_baseline/robot_baseline/紅點成功/2/reward.npy')
    # data1 = np.load('H:/stable_baseline/robot_baseline/藍點成功/2/reward.npy')
    plt.plot(data1)
    plt.show()

def save_done(reward):
    z = reward
    arr = np.load('H:/stable_baseline/robot_baseline/robot/resources/PPO_finish.npy')
    arr = np.append(arr, z)
    np.save('H:/stable_baseline/robot_baseline/robot/resources/PPO_finish.npy', arr)

def load_done():
    data1 = np.load('H:/stable_baseline/robot_baseline/robot/resources/finish.npy')
    # data1 = np.load('H:/stable_baseline/reward/TD3/TD3_reward.npy')
    # data1 = np.load('H:/stable_baseline/robot_baseline/紅點成功/2/reward.npy')
    # data1 = np.load('H:/stable_baseline/robot_baseline/藍點成功/2/reward.npy')
    plt.plot(data1)
    plt.show()

def plt_data():
    data = ['PPO', 'TD3', 'TRPO']
    PPO = np.load('H:/stable_baseline/robot_baseline/robot/resources/PPO_finish.npy')
    TD3 = np.load('H:/stable_baseline/reward/TD3/finish_times.npy')
    TRPO = np.load('H:/stable_baseline/reward/TRPO/finish_times.npy')
    plt.bar(0, PPO.size, 0.3, label="PPO")
    plt.bar(1, TD3.size, 0.3, label="TD3")
    plt.bar(2, TRPO.size, 0.3, label="TRPO")  # 設定標籤
    plt.legend()
    plt.xticks([0, 1, 2], data)
    plt.ylabel('Success times')
    plt.show()


# clear()
# load()
# load_done()

# plt_data()