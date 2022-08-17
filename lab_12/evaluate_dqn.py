import os
import gym
import numpy as np
import matplotlib.pyplot as plt
from dqn_agent import DQNAgent
from utils import reward_engineering_mountain_car
import tensorflow as tf


def plot_points(point_list, style):
    x = []
    y = []
    for point in point_list:
        x.append(point[0])
        y.append(point[1])
    plt.plot(x, y, style)


NUM_EPISODES = 30  # Number of episodes used for evaluation
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

# Creating the DQN agent (with greedy policy, suited for evaluation)
agent = DQNAgent(state_size, action_size, epsilon=0.0, epsilon_min=0.0)

# Checking if weights from previous learning session exists
if os.path.exists('mountain_car.h5'):
    print('Loading weights from previous learning session.')
    agent.load("mountain_car.h5")
else:
    print('No weights found from previous learning session. Unable to proceed.')
    exit(-1)
return_history = []

for episodes in range(1, NUM_EPISODES + 1):
    # Reset the environment
    state = env.reset()
    # This reshape is needed to keep compatibility with Keras
    state = np.reshape(state, [1, state_size])
    # Cumulative reward is the return since the beginning of the episode
    cumulative_reward = 0.0
    for time in range(1, 500):
        # Render the environment for visualization
        env.render()
        # Select action
        action = agent.act(state)
        # Take action, observe reward and new state
        next_state, reward, done, _ = env.step(action)
        # Reshaping to keep compatibility with Keras
        next_state = np.reshape(next_state, [1, state_size])
        # Making reward engineering to keep compatibility with how training was done
        reward = reward_engineering_mountain_car(state[0], action, reward, next_state[0], done)
        state = next_state
        # Accumulate reward
        cumulative_reward = agent.gamma * cumulative_reward + reward
        if done:
            print("episode: {}/{}, time: {}, score: {:.6}, epsilon: {:.3}"
                  .format(episodes, NUM_EPISODES, time, cumulative_reward, agent.epsilon))
            break
    return_history.append(cumulative_reward)

# Prints mean return
print('Mean return: ', np.mean(return_history))

# Plots return history
plt.plot(return_history, 'b')
plt.xlabel('Episode')
plt.ylabel('Return')
plt.savefig('dqn_evaluation.' + fig_format, fig_format=fig_format)

# Plots the greedy policy learned by DQN
plt.figure()
position = np.arange(-1.2, 0.5 + 0.025, 0.05)
velocity = np.arange(-0.07, 0.07 + 0.0025, 0.005)
push_left = []
none = []
push_right = []
for j in range(len(position)):
    for k in range(len(velocity)):
        pos = position[j]
        vel = velocity[k]
        state = np.array([[pos, vel]])
        action = agent.act(state)
        if action == 0:
            push_left.append(state[0])
        elif action == 1:
            none.append(state[0])
        else:
            push_right.append(state[0])
plot_points(push_left, 'b.')
plot_points(none, 'r.')
plot_points(push_right, 'g.')
plt.xlabel('Position')
plt.ylabel('Velocity')
plt.title('Agent Policy')
plt.legend(['Left', 'None', 'Right'])
plt.savefig('agent_decision.' + fig_format, format=fig_format)
plt.show()
