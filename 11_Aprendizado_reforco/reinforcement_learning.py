import numpy as np
import random

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
    # Todo: implement
    num_actions = q.shape[1]  # Número de ações é a segunda dimensão da tabela Q
    if random.random() < epsilon:
        return random.randint(0, num_actions - 1)  # Ação aleatória
    else:
        return np.argmax(q[state, :]) # Ação com maior valor Q para o estado


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
    # Todo: implement
    return np.argmax(q[state, :])


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
        # Todo: implement
        return self.get_exploratory_action(state)

    def learn(self, state, action, reward, next_state, next_action):
        # Todo: implement
        q_next = self.q[next_state, next_action]  # Valor Q do próximo estado e ação
        target = reward + self.gamma * q_next  # Alvo da atualização
        self.q[state, action] += self.alpha * (target - self.q[state, action])  # Atualização


class QLearning(RLAlgorithm):
    def __init__(self, num_states, num_actions, epsilon, alpha, gamma):
        super().__init__(num_states, num_actions, epsilon, alpha, gamma)

    def get_greedy_action(self, state):
        # Todo: implement
        return greedy_action(self.q, state)

    def learn(self, state, action, reward, next_state, next_action):
        # Todo: implement
        q_next = np.max(self.q[next_state, :])  # Maior valor Q no próximo estado
        target = reward + self.gamma * q_next  # Alvo da atualização
        self.q[state, action] += self.alpha * (target - self.q[state, action])  # Atualização
