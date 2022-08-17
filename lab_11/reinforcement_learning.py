import numpy as np


def compute_greedy_policy_as_table(q):
    """
    Computes the greedy policy as a table.

    :param q: action-value table.
    :type q: bidimensional numpy array.
    :return: greedy policy table.
    :rtype: bidimensional numpy array.
    """
    policy = np.zeros(q.shape)
    for s in range(q.shape[0]):
        policy[s, greedy_action(q, s)] = 1.0
    return policy


def epsilon_greedy_action(q, state, epsilon):
    """
    Computes the epsilon-greedy action.

    :param q: action-value table.
    :type q: bidimensional numpy array.
    :param state: current state.
    :type state: int.
    :param epsilon: probability of selecting a random action.
    :type epsilon: float.
    :return: epsilon-greedy action.
    :rtype: int.
    """
    a_chc = np.random.uniform(0, 1)
    a_rng = q.shape[1]
    a_max_idx = np.argmax(q[state, :])
    if a_chc >= epsilon: # Did NOT opt for random action
        return a_max_idx
    else:
        return int(np.random.randint(0, a_rng, 1))


def greedy_action(q, state):
    """
    Computes the greedy action.

    :param q: action-value table.
    :type q: bidimensional numpy array.
    :param state: current state.
    :type state: int.
    :return: greedy action.
    :rtype: int.
    """
    a_max_idx = np.argmax(q[state, :])
    return a_max_idx


class RLAlgorithm:
    """
    Represents a model-free reinforcement learning algorithm.
    """
    def __init__(self, num_states, num_actions, epsilon, alpha, gamma):
        """
        Creates a model-free reinforcement learning algorithm.

        :param num_states: number of states of the MDP.
        :type num_states: int.
        :param num_actions: number of actions of the MDP.
        :type num_actions: int.
        :param epsilon: probability of selecting a random action in epsilon-greedy policy.
        :type epsilon: float.
        :param alpha: learning rate.
        :type alpha: float.
        :param gamma: discount factor.
        :type gamma: float.
        """
        self.q = np.zeros((num_states, num_actions))
        self.epsilon = epsilon
        self.alpha = alpha
        self.gamma = gamma

    def get_num_states(self):
        """
        Returns the number of states of the MDP.

        :return: number of states.
        :rtype: int.
        """
        return self.q.shape[0]

    def get_num_actions(self):
        """
        Returns the number of actions of the MDP.

        :return: number of actions.
        :rtype: int.
        """
        return self.q.shape[1]

    def get_exploratory_action(self, state):
        """
        Returns an exploratory action using epsilon-greedy policy.

        :param state: current state.
        :type state: int.
        :return: exploratory action.
        :rtype: int.
        """
        return epsilon_greedy_action(self.q, state, self.epsilon)

    def get_greedy_action(self, state):
        """
        Returns a greedy action considering the policy of the RL algorithm.

        :param state: current state.
        :type state: int.
        :return: greedy action considering the policy of the RL algorithm.
        :rtype: int.
        """
        raise NotImplementedError('Please implement this method')

    def learn(self, state, action, reward, next_state, next_action):
        raise NotImplementedError('Please implement this method')


class Sarsa(RLAlgorithm):
    def __init__(self, num_states, num_actions, epsilon, alpha, gamma):
        super().__init__(num_states, num_actions, epsilon, alpha, gamma)

    def get_greedy_action(self, state):
        """
        Notice that Sarsa is an on-policy algorithm, so it uses the same epsilon-greedy
        policy for learning and execution.

        :param state: current state.
        :type state: int.
        :return: epsilon-greedy action of Sarsa's execution policy.
        :rtype: int.
        """
        a = epsilon_greedy_action(self.q, state, self.epsilon)
        return a

    def learn(self, state, action, reward, next_state, next_action):
        obj_dist = reward + self.gamma*self.q[next_state, next_action] - self.q[state, action]
        self.q[state, action] = self.q[state, action] + self.alpha * obj_dist
        pass


class QLearning(RLAlgorithm):
    def __init__(self, num_states, num_actions, epsilon, alpha, gamma):
        super().__init__(num_states, num_actions, epsilon, alpha, gamma)

    def get_greedy_action(self, state):
        a = epsilon_greedy_action(self.q, state, self.epsilon)
        return a

    def learn(self, state, action, reward, next_state, next_action):
        obj_dist = reward + self.gamma*np.max(self.q[next_state, :]) - self.q[state, action]
        self.q[state, action] = self.q[state, action] + self.alpha*obj_dist
        pass
