from enum import Enum
import constants
import random
import math


class ExecutionStatus(Enum):
    """
    Represents the execution status of a behavior tree node.
    """
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class BehaviorTree(object):
    """
    Represents a behavior tree.
    """
    def __init__(self, root=None):
        """
        Creates a behavior tree.

        :param root: the behavior tree's root node.
        :type root: TreeNode
        """
        self.root = root

    def update(self, agent):
        """
        Updates the behavior tree.

        :param agent: the agent this behavior tree is being executed on.
        """
        if self.root is not None:
            self.root.execute(agent)


class TreeNode(object):
    """
    Represents a node of a behavior tree.
    """
    def __init__(self, node_name):
        """
        Creates a node of a behavior tree.

        :param node_name: the name of the node.
        """
        self.node_name = node_name
        self.parent = None

    def enter(self, agent):
        """
        This method is executed when this node is entered.

        :param agent: the agent this node is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the behavior tree node logic.

        :param agent: the agent this node is being executed on.
        :return: node status (success, failure or running)
        :rtype: ExecutionStatus
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class LeafNode(TreeNode):
    """
    Represents a leaf node of a behavior tree.
    """
    def __init__(self, node_name):
        super().__init__(node_name)


class CompositeNode(TreeNode):
    """
    Represents a composite node of a behavior tree.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        self.children = []

    def add_child(self, child):
        """
        Adds a child to this composite node.

        :param child: child to be added to this node.
        :type child: TreeNode
        """
        child.parent = self
        self.children.append(child)


class SequenceNode(CompositeNode):
    """
    Represents a sequence node of a behavior tree.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        # We need to keep track of the last running child when resuming the tree execution
        self.running_child = None

    def enter(self, agent):
        # When this node is entered, no child should be running
        self.running_child = None

    def execute(self, agent):
        if self.running_child is None:
            # If a child was not running, then the node puts its first child to run
            self.running_child = self.children[0]
            self.running_child.enter(agent)
        loop = True
        while loop:
            # Execute the running child
            status = self.running_child.execute(agent)
            if status == ExecutionStatus.FAILURE:
                # This is a sequence node, so any failure results in the node failing
                self.running_child = None
                return ExecutionStatus.FAILURE
            elif status == ExecutionStatus.RUNNING:
                # If the child is still running, then this node is also running
                return ExecutionStatus.RUNNING
            elif status == ExecutionStatus.SUCCESS:
                # If the child returned success, then we need to run the next child or declare success
                # if this was the last child
                index = self.children.index(self.running_child)
                if index + 1 < len(self.children):
                    self.running_child = self.children[index + 1]
                    self.running_child.enter(agent)
                else:
                    self.running_child = None
                    return ExecutionStatus.SUCCESS


class SelectorNode(CompositeNode):
    """
    Represents a selector node of a behavior tree.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        # We need to keep track of the last running child when resuming the tree execution
        self.running_child = None

    def enter(self, agent):
        # When this node is entered, no child should be running
        self.running_child = None

    def execute(self, agent):
        if self.running_child is None:
            # If a child was not running, then the node puts its first child to run
            self.running_child = self.children[0]
            self.running_child.enter(agent)
        loop = True
        while loop:
            # Execute the running child
            status = self.running_child.execute(agent)
            if status == ExecutionStatus.FAILURE:
                # This is a selector node, so if the current node failed, we have to try the next one.
                # If there is no child left, then all children failed and the node must declare failure.
                index = self.children.index(self.running_child)
                if index + 1 < len(self.children):
                    self.running_child = self.children[index + 1]
                    self.running_child.enter(agent)
                else:
                    self.running_child = None
                    return ExecutionStatus.FAILURE
            elif status == ExecutionStatus.RUNNING:
                # If the child is still running, then this node is also running
                return ExecutionStatus.RUNNING
            elif status == ExecutionStatus.SUCCESS:
                # If any child returns success, then this node must also declare success
                self.running_child = None
                return ExecutionStatus.SUCCESS


class RoombaBehaviorTree(BehaviorTree):
    """
    Represents a behavior tree of a roomba cleaning robot.
    """
    def __init__(self):
        super().__init__()
        MoveSequence = SequenceNode("MoveSequence")
        MoveSequence.add_child(MoveForwardNode())
        MoveSequence.add_child(MoveInSpiralNode())
        WallSequence = SequenceNode("WallSequence")
        WallSequence.add_child(GoBackNode())
        WallSequence.add_child(RotateNode())
        RootSelector = SelectorNode("RootSelector")
        RootSelector.add_child(MoveSequence)
        RootSelector.add_child(WallSequence)
        self.root = RootSelector


class MoveForwardNode(LeafNode):
    def __init__(self):
        super().__init__("MoveForward")
        self.sampleTime = constants.SAMPLE_TIME
        self.linearSpeed = constants.FORWARD_SPEED
        self.angularSpeed = 0
        self.t2 = constants.MOVE_FORWARD_TIME
        self.t = 0

    def enter(self, agent):
        self.t = 0
        pass

    def execute(self, agent):
        self.t = self.t + self.sampleTime
        agent.set_velocity(linear_speed=self.linearSpeed, angular_speed=self.angularSpeed)
        if self.t < self.t2 and (not agent.get_bumper_state()):
            return ExecutionStatus.RUNNING
        elif self.t >= self.t2 and (not agent.get_bumper_state()):
            return ExecutionStatus.SUCCESS
        else:
            return ExecutionStatus.FAILURE
        pass


class MoveInSpiralNode(LeafNode):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.sampleTime = constants.SAMPLE_TIME
        self.radius = constants.INITIAL_RADIUS_SPIRAL
        self.linearSpeed = constants.FORWARD_SPEED
        self.angularSpeed = self.linearSpeed / self.radius
        self.b = constants.SPIRAL_FACTOR
        self.t1 = constants.MOVE_IN_SPIRAL_TIME
        self.t = 0

    def enter(self, agent):
        self.t = 0
        self.radius = constants.INITIAL_RADIUS_SPIRAL
        self.angularSpeed = self.linearSpeed / self.radius
        pass

    def execute(self, agent):
        self.t = self.t + self.sampleTime
        self.radius = constants.INITIAL_RADIUS_SPIRAL + self.t*self.b
        self.angularSpeed = self.linearSpeed/self.radius
        agent.set_velocity(linear_speed=self.linearSpeed, angular_speed=self.angularSpeed)
        if self.t < self.t1 and (not agent.get_bumper_state()):
            return ExecutionStatus.RUNNING
        elif self.t >= self.t1 and (not agent.get_bumper_state()):
            return ExecutionStatus.SUCCESS
        else:
            return ExecutionStatus.FAILURE
        pass


class GoBackNode(LeafNode):
    def __init__(self):
        super().__init__("GoBack")
        self.sampleTime = constants.SAMPLE_TIME
        self.linearSpeed = constants.BACKWARD_SPEED
        self.angularSpeed = 0
        self.t3 = constants.GO_BACK_TIME
        self.t = 0

    def enter(self, agent):
        self.t = 0
        pass

    def execute(self, agent):
        self.t = self.t + self.sampleTime
        agent.set_velocity(linear_speed=self.linearSpeed, angular_speed=self.angularSpeed)
        if self.t < self.t3:
            return ExecutionStatus.RUNNING
        elif self.t >= self.t3:
            return ExecutionStatus.SUCCESS
        pass


class RotateNode(LeafNode):
    def __init__(self):
        super().__init__("Rotate")
        self.sampleTime = constants.SAMPLE_TIME
        self.angle = 0
        self.randomAngle = random.uniform(-math.pi, math.pi)
        self.angularSpeed = math.copysign(constants.ANGULAR_SPEED, self.randomAngle)
        self.linearSpeed = 0
        self.t = 0

    def enter(self, agent):
        self.t = 0
        self.angle = 0
        self.randomAngle = random.uniform(-math.pi, math.pi)
        self.angularSpeed = math.copysign(constants.ANGULAR_SPEED, self.randomAngle)
        pass

    def execute(self, agent):
        self.t = self.t + self.sampleTime
        self.angle = self.angle + self.sampleTime*self.angularSpeed
        agent.set_velocity(linear_speed=self.linearSpeed, angular_speed=self.angularSpeed)
        if abs(self.angle) < abs(self.randomAngle):
            return ExecutionStatus.RUNNING
        else:
            return ExecutionStatus.SUCCESS
        pass

