START_POSITION_CAR = -0.5


def reward_engineering_mountain_car(state, action, reward, next_state, done):
    """
    Makes reward engineering to allow faster training in the Mountain Car environment.

    :param state: state.
    :type state: NumPy array with dimension (1, 2).
    :param action: action.
    :type action: int.
    :param reward: original reward.
    :type reward: float.
    :param next_state: next state.
    :type next_state: NumPy array with dimension (1, 2).
    :param done: if the simulation is over after this experience.
    :type done: bool.
    :return: modified reward for faster training.
    :rtype: float.
    """
        reward = reward + (state[0]-START_POSITION_CAR)**2 + state[1]**2
        eig = 0
        if next_state[0] >= 0.5:
            eig = 1
        reward = reward + 50*eig
    return reward


