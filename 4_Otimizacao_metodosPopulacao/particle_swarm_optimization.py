import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """

        # Inicializa a posição aleatoriamente dentro dos limites
        self.position = np.array([random.uniform(low, high)
                                  for low, high in zip(lower_bound, upper_bound)])

        # Inicializa a velocidade com zeros ou valores pequenos aleatórios
        delta = upper_bound - lower_bound
        self.velocity = np.array([random.uniform(-d, d) for d in delta])

        # Melhor posição individual (inicialmente é a posição atual)
        self.best_position = np.copy(self.position)

        # Valor da melhor posição individual (inicialmente menos infinito)
        self.best_value = -inf


class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):

        self.w = hyperparams.inertia_weight
        self.phi_p = hyperparams.cognitive_parameter
        self.phi_g = hyperparams.social_parameter
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

        # Controla qual partícula está sendo avaliada
        self.current_particle_index = 0

        # Inicializa o enxame de partículas
        self.particles = [Particle(lower_bound, upper_bound)
                          for _ in range(hyperparams.num_particles)]

        # Melhor posição global e seu valor
        self.global_best_position = None
        self.global_best_value = -inf

    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        return np.copy(self.global_best_position)


    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        return self.global_best_value


    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        # Todo: implement
        # retorna as posições de todas as particulas do enxame
        current_particle = self.particles[self.current_particle_index]
        return np.copy(current_particle.position)

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        # Todo: implement
        for particle in self.particles:
            # Gera números aleatórios para os componentes cognitivo e social
            r_p = np.random.uniform(0, 1, size=len(particle.position))
            r_g = np.random.uniform(0, 1, size=len(particle.position))

            # Calcula os componentes da velocidade
            cognitive = self.phi_p * r_p * (particle.best_position - particle.position)
            social = self.phi_g * r_g * (self.global_best_position - particle.position)

            # Atualiza a velocidade
            particle.velocity = self.w * particle.velocity + cognitive + social

            # Atualiza a posição
            particle.position += particle.velocity

            # Garante que a posição está dentro dos limites
            particle.position = np.clip(particle.position, self.lower_bound, self.upper_bound)

        self.generation_completed = True

    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        current_particle = self.particles[self.current_particle_index]

        # Atualiza o melhor pessoal (pBest)
        if value > current_particle.best_value:
            current_particle.best_value = value
            current_particle.best_position = np.copy(current_particle.position)

        # Atualiza o melhor global (gBest)
        if value > self.global_best_value:
            self.global_best_value = value
            self.global_best_position = np.copy(current_particle.position)

        # Avança para a próxima partícula
        self.current_particle_index += 1

        # Se todas as partículas foram avaliadas, avança a geração
        if self.current_particle_index >= len(self.particles):
            self.advance_generation()
            self.current_particle_index = 0
            self.generation_completed = False


