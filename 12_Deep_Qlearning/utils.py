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
    # Todo: implement reward engineering
    # Extract position and velocity from state
    position = state[0]
    velocity = state[1]
    # Extract next position
    next_position = next_state[0]

    # Compute modified reward
    modified_reward = reward + (position - START_POSITION_CAR) ** 2 + abs(velocity)*10

    # Add bonus if goal is reached (next_position >= 0.5)
    if next_position >= 0.5:
        modified_reward += 50
    if next_position >= 0.3:
        modified_reward += 0.2
    if next_position >= 0.1:
        modified_reward += 0.1


    return modified_reward


