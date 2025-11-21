from queue import PriorityQueue
import copy
import math
import itertools
from numbers import Integral


class Node:
    def __init__(self, position):
        self.pos = position
        self.cost = 0.0
        self.heuristic = 0.0
        self.parent = None


class AStarPlanner:
    # Initialize any variables
    def __init__(self, safe_distance=1):
        # Save the safe_distance
        self.safe_distance = safe_distance

    # Find a plan through the map from drone_position to goal_position
    def plan(self, map_data, drone_position, goal_position):
        # Validate the data
        self.validate_data(map_data, drone_position, goal_position)

        # Expand the obstacles by the safety factor
        map_data = self.expand_obstacles(map_data, self.safe_distance)

        # Create the goal_node
        goal_n = Node(goal_position)

        # Create the already processed
        closed = []

        # Create the frontier
        frontier = PriorityQueue()

        # Tiebreaker counter so (priority, counter, node) is always comparable
        counter = itertools.count()

        # Add the start node to the open list
        cur_node = Node(drone_position)
        cur_node.heuristic = self.movement_cost(cur_node, goal_n)
        frontier.put((cur_node.heuristic + cur_node.cost, next(counter), cur_node))

        # While the frontier is not empty
        while not frontier.empty():
            # Get the node with the lowest cost
            _, _, cur_node = frontier.get()

            # Check if the current node is the goal
            if (cur_node.pos[0] == goal_n.pos[0]) and (cur_node.pos[1] == goal_n.pos[1]):
                final_path = []
                while cur_node.parent is not None:
                    final_path.append(cur_node.pos)
                    cur_node = cur_node.parent
                # Save your starting position
                final_path.append(cur_node.pos)
                # Traverse backwards along parent nodes
                return list(reversed(final_path))

            # For each neighbor
            neighbors = self.get_neighbors(cur_node, map_data)
            for neighbor in neighbors:
                # Compute the cost to traverse to that node
                new_cost = cur_node.cost + self.movement_cost(cur_node, neighbor)

                # If the neighbor is inside the frontier and new_cost < neighbor.cost
                frontier_list = self.priorityQueueToList(frontier)
                index = self.insidelist(neighbor, frontier_list)
                if index != -1:
                    # frontier.queue items are (priority, counter, node)
                    if new_cost < frontier.queue[index][2].cost:
                        # Remove from frontier
                        frontier.queue.pop(index)

                # If the neighbor is inside the closed list and new_cost < neighbor.cost
                index = self.insidelist(neighbor, closed)
                if index != -1:
                    if new_cost < closed[index].cost:
                        # Remove from closed
                        closed.pop(index)

                # If neighbor is not inside either
                if (self.insidelist(neighbor, frontier_list) == -1) and (self.insidelist(neighbor, closed) == -1):
                    neighbor.cost = new_cost
                    neighbor.heuristic = self.movement_cost(neighbor, goal_n)
                    frontier.put((neighbor.heuristic + neighbor.cost, next(counter), neighbor))
                    neighbor.parent = cur_node

            # Add current node to closed after expanding neighbors
            if self.insidelist(cur_node, closed) == -1:
                closed.append(cur_node)

        # If no path is found, return None (or raise)
        return None

    # Compute the movement cost
    def movement_cost(self, n1, n2):
        return math.hypot(n1.pos[0] - n2.pos[0], n1.pos[1] - n2.pos[1])

    # Return the index of the node inside this list
    # If the node is not in the list return -1
    def insidelist(self, node_in, list_in):
        for i, v in enumerate(list_in):
            if (v.pos[0] == node_in.pos[0]) and (v.pos[1] == node_in.pos[1]):
                return i
        return -1

    # Return the priority queue's data as a list of nodes
    def priorityQueueToList(self, queue_in):
        # Underlying list holds tuples; the node is at index 2
        return [item[2] for item in queue_in.queue]

    # Get the children nodes for the current node
    def get_neighbors(self, node_in, map_in):
        neighbors = []
        pos = node_in.pos
        # For all adjacent values
        for x_dim in range(-1, 2):
            for y_dim in range(-1, 2):
                if not (x_dim == 0 and y_dim == 0):
                    ni, nj = pos[0] + x_dim, pos[1] + y_dim
                    if (0 <= ni < map_in.shape[0]) and (0 <= nj < map_in.shape[1]):
                        if map_in[ni][nj] == 0:
                            n = Node([ni, nj])
                            n.heuristic = 0.0  # will be set before pushing
                            neighbors.append(n)
        return neighbors

    # Validate the incoming data
    def validate_data(self, map_data, drone_position, goal_position):
        # Confirm that the map has two dimensions
        assert len(map_data.shape) == 2
        # Confirm that the drone and goal position are two points
        assert len(drone_position) == 2
        assert len(goal_position) == 2
        # Confirm that all positions are integers
        for x in drone_position + goal_position:
            assert isinstance(x, Integral)
        # Confirm that both the start and end goal lie inside the map
        assert 0 <= drone_position[0] < map_data.shape[0]
        assert 0 <= drone_position[1] < map_data.shape[1]
        assert 0 <= goal_position[0] < map_data.shape[0]
        assert 0 <= goal_position[1] < map_data.shape[1]
        # Confirm that the drone_position and the goal position are not the same
        # assert not (drone_position[0] == goal_position[0] and drone_position[1] == goal_position[1])
        # Confirm that the goal and drone position are free space
        assert map_data[drone_position[0], drone_position[1]] == 0
        assert map_data[goal_position[0], goal_position[1]] == 0

    # Expand the obstacles by distance so you do not hit one
    def expand_obstacles(self, map_data, distance):
        new_map = copy.deepcopy(map_data)
        # For each element in the map
        for i in range(map_data.shape[0]):
            for j in range(map_data.shape[1]):
                # if this is an obstacle
                if map_data[i, j] != 0:
                    # Expand the obstacle by 1 in all directions
                    for x_dim in range(-distance, distance + 1):
                        for y_dim in range(-distance, distance + 1):
                            ni, nj = i + x_dim, j + y_dim
                            if (0 <= ni < new_map.shape[0]) and (0 <= nj < new_map.shape[1]):
                                new_map[ni, nj] = map_data[i, j]
        return new_map
