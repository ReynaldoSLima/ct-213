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
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.x = np.random.uniform(self.lower_bound, self.upper_bound)
        delta = self.upper_bound - self.lower_bound
        self.v = np.random.uniform(-delta, delta)
        self.best = self.x
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
        self.num_particles = hyperparams.num_particles
        self.inertia_weight = hyperparams.inertia_weight
        self.cognitive_parameter = hyperparams.cognitive_parameter
        self.social_parameter = hyperparams.social_parameter
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.v_min = - (self.upper_bound - self.lower_bound)
        self.v_max = self.upper_bound - self.lower_bound
        self.particles = []
        for i in range(self.num_particles):
            particle = Particle(lower_bound=self.lower_bound, upper_bound=self.upper_bound)
            self.particles.append(particle)
        self.best_global = None
        self.best_iteration = None
        self.best_global_value = -inf
        self.best_iteration_value = -inf
        self.current_particle = 0

    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        return self.best_global

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        return self.best_global_value

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        if self.best_global is None:
            return self.particles[self.current_particle].x
        else:
            particle = self.particles[self.current_particle]
            rp = random.uniform(0.0, 1.0)
            rg = random.uniform(0.0, 1.0)
            particle.v = self.inertia_weight * particle.v + self.cognitive_parameter * rp * (
                    particle.best - particle.x) + self.social_parameter * rg * (self.best_global - particle.x)
            particle.x = particle.x + particle.v

            self.particles[self.current_particle].v = particle.v
            self.particles[self.current_particle].x = particle.x
        return self.particles[self.current_particle].x

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        self.current_particle = 0
        if self.best_iteration_value > self.best_global_value:
            self.best_global = self.best_iteration
            self.best_global_value = self.best_iteration_value

    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        if value > self.particles[self.current_particle].best_value:
            self.particles[self.current_particle].best = self.particles[self.current_particle].x
            self.particles[self.current_particle].best_value = value
        if value > self.best_iteration_value:
            self.best_iteration = self.particles[self.current_particle].x
            self.best_iteration_value = value
        self.current_particle += 1
        if self.current_particle == self.num_particles:
            self.advance_generation()
