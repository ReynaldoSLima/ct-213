import random
import math
import constants


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        self.sampleTime = constants.SAMPLE_TIME
        self.linearSpeed = constants.FORWARD_SPEED
        self.angularSpeed = 0
        self.t2 = constants.MOVE_FORWARD_TIME
        self.t = 0

    def check_transition(self, agent, state_machine):
        # Utiliza o binario do bumper para saber se encontrou parede
        if agent.get_bumper_state():
            state_machine.change_state(GoBackState())
        # Se passou do tempo t2 = 3.0 s, passar para mover em espiral
        elif self.t > self.t2:
            state_machine.change_state(MoveInSpiralState())
        pass

    def execute(self, agent):
        # Atualiza o tempo e aplica a velocidade desejada (angular sendo nula, linear sendo a padrão)
        self.t = self.t + self.sampleTime
        agent.set_velocity(linear_speed=self.linearSpeed, angular_speed=self.angularSpeed)
        pass


class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.sampleTime = constants.SAMPLE_TIME
        self.radius = constants.INITIAL_RADIUS_SPIRAL
        self.linearSpeed = constants.FORWARD_SPEED
        self.angularSpeed = self.linearSpeed/self.radius
        self.b = constants.SPIRAL_FACTOR
        self.t1 = constants.MOVE_IN_SPIRAL_TIME
        self.t = 0
    
    def check_transition(self, agent, state_machine):
        if agent.get_bumper_state():
            state_machine.change_state(GoBackState())
        elif self.t > self.t1:
            state_machine.change_state(MoveForwardState())
        pass

    def execute(self, agent):
        self.t = self.t + self.sampleTime
        self.radius = constants.INITIAL_RADIUS_SPIRAL + self.t*self.b
        self.angularSpeed = self.linearSpeed/self.radius
        agent.set_velocity(linear_speed=self.linearSpeed, angular_speed=self.angularSpeed)
        pass


class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.sampleTime = constants.SAMPLE_TIME
        self.linearSpeed = constants.BACKWARD_SPEED
        self.angularSpeed = 0
        self.t3 = constants.GO_BACK_TIME
        self.t = 0

    def check_transition(self, agent, state_machine):
        if self.t > self.t3:
            state_machine.change_state(RotateState())
        pass

    def execute(self, agent):
        self.t = self.t + self.sampleTime
        agent.set_velocity(linear_speed=self.linearSpeed, angular_speed=self.angularSpeed)
        pass


class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        self.sampleTime = constants.SAMPLE_TIME
        self.angle = 0
        self.randomAngle = random.uniform(-math.pi, math.pi)
        self.angularSpeed = math.copysign(constants.ANGULAR_SPEED, self.randomAngle)
        self.linearSpeed = 0
        self.t = 0

    def check_transition(self, agent, state_machine):
        if abs(self.angle) >= abs(self.randomAngle):
            state_machine.change_state(MoveForwardState())
        pass
    
    def execute(self, agent):
        self.t = self.t + self.sampleTime
        self.angle = self.angle + self.sampleTime*self.angularSpeed
        agent.set_velocity(linear_speed=self.linearSpeed, angular_speed=self.angularSpeed)
        pass
