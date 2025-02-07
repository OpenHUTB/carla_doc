from osim.env import ProstheticsEnv
env = ProstheticsEnv(visualize=True)
observation = env.reset()
for i in range(200):
    o, r, d, i = env.step(env.action_space.sample())