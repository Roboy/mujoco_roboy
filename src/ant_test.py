import numpy as np
import gym
import time

env = gym.make('Ant-v2')
env.reset()

for i in range(200):
    env.step(np.zeros(1))
    env.render()
    time.sleep(0.01)
