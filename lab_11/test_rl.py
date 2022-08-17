import numpy as np
from reinforcement_learning import Sarsa, QLearning, greedy_action


def dynamics(state, action, num_states):
    """
    Defines the dynamics of the simple deterministic corridor MDP.
    The corridor considers a wrap to the other side of the corridor
    when one side is reached. If a left move is issued when the
    agent is on the leftmost cell, then it appears on the rightmost cell.
    If a right move is executed when the agent is on the rightmost cell,
    then it appears on the leftmost cell.


    :param state: current state.
    :type state: int.
    :param action: chosen action.
    :type action: int.
    :param num_states: number of states of the MDP.
    :type num_states: int.
    :return: next state.
    :rtype: int.
    """
    if action == STOP:
        return state
    elif action == LEFT:
        return (state - 1) % num_states
    elif action == RIGHT:
        return (state + 1) % num_states


def reward_signal(state, action, num_states):
    """
    Defines the reward signal of the simple deterministic corridor MDP.

    :param state: current state.
    :type state: int.
    :param action: chosen action.
    :type action: int.
    :param num_states: number of states of the MDP.
    :type num_states: int.
    :return: reward.
    :rtype: float.
    """
    if state != num_states - 1:  # the rightmost cell is the goal state
        return -1.0
    return 0.0


def print_greedy_policy(q):
    """
    Prints the greedy policy evaluated using a action-value table.

    :param q: action-value table.
    :type q: bidimensional numpy array.
    """
    actions_names = ['S', 'L', 'R']
    num_states = q.shape[0]
    print('[', end='')
    for s in range(num_states):
        action = greedy_action(q, s)
        print(actions_names[action], end='')
        if s != num_states - 1:
            print(', ', end='')
        else:
            print(']')


# Possible actions
STOP = 0
LEFT = 1
RIGHT = 2

num_states = 10  # corridor with 10 cells
num_actions = 3  # STOP, LEFT or RIGHT
epsilon = 0.1  # epsilon of epsilon-greedy
alpha = 0.1  # learning rate
gamma = 0.99  # discount factor
num_episodes = 1000  # number of episodes used in learning
num_iterations = 1000  # number of steps per episode
# Todo: comment/uncomment the lines below to select the desired algorithm
rl_algorithm = Sarsa(num_states, num_actions, epsilon, alpha, gamma)
#rl_algorithm = QLearning(num_states, num_actions, epsilon, alpha, gamma)

for i in range(num_episodes):
    state = np.random.randint(0, num_states)
    action = rl_algorithm.get_exploratory_action(state)
    for j in range(num_iterations):
        next_state = dynamics(state, action, num_states)
        reward = reward_signal(state, action, num_states)
        next_action = rl_algorithm.get_exploratory_action(next_state)
        rl_algorithm.learn(state, action, reward, next_state, next_action)
        state = next_state
        action = next_action

print('Action-value Table:')
print(rl_algorithm.q)
print('Greedy policy learnt:')
print_greedy_policy(rl_algorithm.q)
