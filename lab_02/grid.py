import numpy as np
import random
from math import inf, sqrt


class CostMap(object):
    """
    Represents a cost map where higher values indicates terrain which are harder to transverse.
    """
    def __init__(self, width, height):
        """
        Creates a cost map.

        :param width: width (number of columns) of the cost map.
        :type width: int.
        :param height: height (number of rows) of the cost map.
        :type height: int.
        """
        self.width = width
        self.height = height
        self.grid = np.ones((height, width))

    def get_cell_cost(self, i, j):
        """
        Obtains the cost of a cell in the cost map.

        :param i: the row (y coordinate) of the cell.
        :type i: int.
        :param j: the column (x coordinate) of the cell.
        :type j: int.
        :return: cost of the cell.
        :rtype: float.
        """
        return self.grid[i, j]

    def get_edge_cost(self, start, end):
        """
        Obtains the cost of an edge.

        :param start: tbe cell where the edge starts.
        :type start: float.
        :param end: the cell where the edge ends.
        :type end: float.
        :return: cost of the edge.
        :rtype: float.
        """
        diagonal = (start[0] != end[0]) and (start[1] != end[1])
        factor = sqrt(2) if diagonal else 1.0
        return factor * (self.get_cell_cost(start[0], start[1]) + self.get_cell_cost(end[0], end[1])) / 2.0

    def is_occupied(self, i, j):
        """
        Checks if a cell is occupied.

        :param i: the row of the cell.
        :type i: int.
        :param j: the column of the cell.
        :type j: int.
        :return: True if the cell is occupied, False otherwise.
        :rtype: bool.
        """
        return self.grid[i][j] < 0.0

    def is_index_valid(self, i, j):
        """
        Check if a (i,j) position is valid (is within the map boundaries).

        :param i: the row of the cell.
        :param i: int.
        :param j: the column of the cell.
        :param j: int.
        :return: if the index is valid.
        :rtype: bool.
        """
        return 0 <= i < self.height and 0 <= j < self.width

    def add_random_obstacle(self, width, height):
        """
        Adds a random obstacle to the map.

        :param width: width (number of columns) of the obstacle.
        :type width: int.
        :param height: height (number of rows) of the obstacle.
        :type height: int.
        """
        top_left = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
        self.add_obstacle((top_left[0], top_left[1], width, height))

    def add_obstacle(self, rectangle):
        """
        Adds an obstacle given a rectangular region (x, y, width, height).

        :param rectangle: a rectangle defined as (x, y, width, height), where (x, y) is the top left corner.
        :type rectangle: 4-dimensional tuple.
        """
        self.add_rectangle((rectangle[0] - 1, rectangle[1] - 1, rectangle[2] + 2, rectangle[3] + 2), 2.0)
        self.add_rectangle(rectangle, -1.0)

    def add_rectangle(self, rectangle, value):
        """
        Changes the values of a rectangular region to a given value.

        :param rectangle: rectangular region defined as (x, y, width, height), where (x, y) is the top left corner.
        :param value: the value used in the rectangular region.
        """
        left = rectangle[0]
        right = rectangle[0] + rectangle[2]
        top = rectangle[1]
        bottom = rectangle[1] + rectangle[3]
        for j in range(left, right):
            for i in range(top, bottom):
                if self.is_index_valid(i, j) and not self.is_occupied(i, j):
                    self.grid[i, j] = value

    def create_random_map(self, obstacle_width, obstacle_height, num_obstacles):
        """
        Creates a random map by creating many random obstacles.

        :param obstacle_width: width (number of columns) of each obstacle.
        :type obstacle_width: int.
        :param obstacle_height: height (number of rows) of each obstacle.
        :type obstacle_height: int.
        :param num_obstacles: number of obstacles.
        :type num_obstacles: int.
        """
        for i in range(num_obstacles):
            self.add_random_obstacle(obstacle_width, obstacle_height)


class NodeGrid(object):
    """
    Represents a grid of graph nodes used by the planning algorithms.
    """
    def __init__(self, cost_map):
        """
        Creates a grid of graph nodes.

        :param cost_map: cost map used for planning.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.width = cost_map.width
        self.height = cost_map.height
        self.grid = np.empty((self.height, self.width), dtype=Node)
        for i in range(np.size(self.grid, 0)):
            for j in range(np.size(self.grid, 1)):
                self.grid[i, j] = Node(i, j)

    def reset(self):
        """
        Resets all nodes of the grid.
        """
        for row in self.grid:
            for node in row:
                node.reset()

    def get_node(self, i, j):
        """
        Obtains the node at row i and column j.

        :param i: row of the node.
        :type i: int.
        :param j: column of the node.
        :type j: int.
        :return: node at row i and column j.
        :rtype: Node.
        """
        return self.grid[i, j]

    def get_successors(self, i, j):
        """
        Obtains a list of the 8-connected successors of the node at (i, j).

        :param i: row of the node.
        :type i: int.
        :param j: column of the node.
        :type j: int.
        :return: list of the 8-connected successors.
        :rtype: list of Node.
        """
        successors = []
        for di in range(-1, 2):
            for dj in range(-1, 2):
                if di != 0 or dj != 0:
                    if self.cost_map.is_index_valid(i + di, j + dj) and not self.cost_map.is_occupied(i + di, j + dj):
                        successors.append((i + di, j + dj))
        return successors


class Node(object):
    """
    Represents a node of a graph used for planning paths.
    """
    def __init__(self, i=0, j=0):
        """
        Creates a node of a graph used for planning paths.

        :param i: row of the node in the occupancy grid.
        :type i: int.
        :param j: column of the node in the occupancy grid.
        :type j: int.
        """
        self.i = i
        self.j = j
        self.f = inf
        self.g = inf
        self.closed = False
        self.parent = None

    def get_position(self):
        """
        Obtains the position of the node as a tuple.

        :return: (i, j) where i is the row and the column of the node, respectively.
        :rtype: 2-dimensional tuple of int.
        """
        return self.i, self.j

    def set_position(self, i, j):
        """
        Sets the position of this node.

        :param i: row of the node in the occupancy grid.
        :type i: int.
        :param j: column of the node in the occupancy grid.
        :type j: int.
        """
        self.i = i
        self.j = j

    def reset(self):
        """
        Resets the node to prepare it for a new path planning.
        """
        self.f = inf
        self.g = inf
        self.closed = False
        self.parent = None

    def distance_to(self, i, j):
        """
        Computes the distance from this node to the position (i, j).

        :param i: row of the target position.
        :type i: int.
        :param j: column of the target position.
        :type j: int.
        :return: distance from this node to (i, j).
        :rtype: float.
        """
        return sqrt((self.i - i) ** 2 + (self.j - j) ** 2)

    def __lt__(self, another_node):
        if self.i < another_node.i:
            return True
        if self.j < another_node.j:
            return True
        return False

