from grid import Node, NodeGrid
from math import inf, sqrt
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """

    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        pq = []
        start = self.node_grid.get_node(start_position[0], start_position[1])
        goal = self.node_grid.get_node(goal_position[0], goal_position[1])
        start.g = 0
        heapq.heappush(pq, (start.g, start))
        while not (len(pq) == 0):
            g, node_temp = heapq.heappop(pq)
            if not self.node_grid.get_node(node_temp.i, node_temp.j).closed:
                node = node_temp
                node.closed = True
            else:
                continue
            if node.i == goal_position[0] and node.j == goal_position[1]:
                break
            for (successor_i, successor_j) in self.node_grid.get_successors(node.i, node.j):
                successor = self.node_grid.get_node(successor_i, successor_j)
                diagonal = (node.i != successor.i) and (node.j != successor.j)
                factor = sqrt(2) if diagonal else 1.0
                cost_i_to_j = factor * (self.cost_map.get_cell_cost(successor.i, successor.j) +
                                        self.cost_map.get_cell_cost(node.i, node.j)) / 2
                if successor.g > node.g + cost_i_to_j and (not successor.closed):
                    successor.g = node.g + cost_i_to_j
                    successor.parent = node
                    heapq.heappush(pq, (successor.g, successor))
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        goal_cost = goal_node.g
        goal_sequence = self.construct_path(goal_node)
        self.node_grid.reset()
        return goal_sequence, goal_cost

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        pq = []
        start = self.node_grid.get_node(start_position[0], start_position[1])
        goal = self.node_grid.get_node(goal_position[0], goal_position[1])
        start.g = 0
        start.f = start.distance_to(goal.i, goal.j)
        heapq.heappush(pq, (start.f, start))
        while not (len(pq) == 0):
            f, node_temp = heapq.heappop(pq)
            if not self.node_grid.get_node(node_temp.i, node_temp.j).closed:
                node = node_temp
                node.closed = True
            else:
                continue
            if node.i == goal.i and node.j == goal.j:
                break
            for (successor_i, successor_j) in self.node_grid.get_successors(node.i, node.j):
                successor = self.node_grid.get_node(successor_i, successor_j)
                diagonal = (node.i != successor.i) and (node.j != successor.j)
                factor = sqrt(2) if diagonal else 1.0
                cost_i_to_j = factor * (self.cost_map.get_cell_cost(successor.i, successor.j) +
                                        self.cost_map.get_cell_cost(node.i, node.j)) / 2
                h = successor.distance_to(goal.i, goal.j)
                if not successor.closed:
                    successor.g = node.g + cost_i_to_j
                    successor.f = h
                    successor.parent = node
                    heapq.heappush(pq, (successor.f, successor))
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        goal_cost = goal_node.g
        goal_sequence = self.construct_path(goal_node)
        self.node_grid.reset()
        return goal_sequence, goal_cost

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        pq = []
        start = self.node_grid.get_node(start_position[0], start_position[1])
        goal = self.node_grid.get_node(goal_position[0], goal_position[1])
        start.g = 0
        start.f = start.g + start.distance_to(goal.i, goal.j)
        heapq.heappush(pq, (start.f, start))
        while not (len(pq) == 0):
            f, node_temp = heapq.heappop(pq)
            if not self.node_grid.get_node(node_temp.i, node_temp.j).closed:
                node = node_temp
                node.closed = True
            else:
                continue
            if node.i == goal.i and node.j == goal.j:
                break
            for (successor_i, successor_j) in self.node_grid.get_successors(node.i, node.j):
                successor = self.node_grid.get_node(successor_i, successor_j)
                diagonal = (node.i != successor.i) and (node.j != successor.j)
                factor = sqrt(2) if diagonal else 1.0
                cost_i_to_j = factor * (self.cost_map.get_cell_cost(successor.i, successor.j) +
                                        self.cost_map.get_cell_cost(node.i, node.j)) / 2
                h = successor.distance_to(goal.i, goal.j)
                if successor.f > node.g + cost_i_to_j + h and (not successor.closed):
                    successor.g = node.g + cost_i_to_j
                    successor.f = successor.g + h
                    successor.parent = node
                    heapq.heappush(pq, (successor.f, successor))
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        goal_cost = goal_node.g
        goal_sequence = self.construct_path(goal_node)
        self.node_grid.reset()
        return goal_sequence, goal_cost
