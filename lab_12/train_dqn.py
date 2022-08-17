import os
import gym
import numpy as np
import matplotlib.pyplot as plt
from dqn_agent import DQNAgent
from utils import reward_engineering_mountain_car
import tensorflow as tf


NUM_EPISODES = 300  # Number of episodes used for training
RENDER = False  # If the Mountain Car environment should be rendered
fig_format = 'png'  # Format used for saving matplotlib's figures
# fig_format = 'eps'
# fig_format = 'svg'

# Comment this line to enable training using your GPU
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

tf.compat.v1.disable_eager_execution()

# Initiating the Mountain Car environment
env = gym.make('MountainCar-v0')
state_size = env.observation_space.shape[0]
action_size = env.action_space.n

# Creating the DQN agent
agent = DQNAgent(state_size, action_size)

# Checking if weights from previous learning session exists
if os.path.exists('mountain_car.h5'):
    print('Loading weights from previous learning session.')
    agent.load("mountain_car.h5")
else:
    print('No weights found from previous learning session.')
done = False
batch_size = 32  # batch size used for the experience replay
return_history = []

for episodes in range(1, NUM_EPISODES + 1):
    # Reset the environment
    state = env.reset()
    # This reshape is needed to keep compatibility with Keras
    state = np.reshape(state, [1, state_size])
    # Cumulative reward is the return since the beginning of the episode
    cumulative_reward = 0.0
    for time in range(1, 500):
        if RENDER:
            env.render()  # Render the environment for visualization
        # Select action
        action = agent.act(state)
        # Take action, observe reward and new state
        next_state, reward, done, _ = env.step(action)
        # Reshaping to keep compatibility with Keras
        next_state = np.reshape(next_state, [1, state_size])
        # Making reward engineering to allow faster training
        reward = reward_engineering_mountain_car(state[0], action, reward, next_state[0], done)
        # Appending this experience to the experience replay buffer
        agent.append_experience(state, action, reward, next_state, done)
        state = next_state
        # Accumulate reward
        cumulative_reward = agent.gamma * cumulative_reward + reward
        if done:
            print("episode: {}/{}, time: {}, score: {:.6}, epsilon: {:.3}"
                  .format(episodes, NUM_EPISODES, time, cumulative_reward, agent.epsilon))
            break
        # We only update the policy if we already have enough experience in memory
        if len(agent.replay_buffer) > 2 * batch_size:
            loss = agent.replay(batch_size)
    return_history.append(cumulative_reward)
    agent.update_epsilon()
    # Every 10 episodes, update the plot for training monitoring
    if episodes % 20 == 0:
        plt.plot(return_history, 'b')
        plt.xlabel('Episode')
        plt.ylabel('Return')
        plt.show(block=False)
        plt.pause(0.1)
        plt.savefig('dqn_training.' + fig_format, fig_format=fig_format)
        # Saving the model to disk
        agent.save("mountain_car.h5")
plt.pause(1.0)
