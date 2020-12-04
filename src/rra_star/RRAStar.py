import numpy as np
import heapq
import copy
from rra_star.node import Node
from rra_star.utils import DistanceMethod, calculate_distance, get_neighbors_reverse
from rra_star.visualization import Graph


class RRAStar(object):
    def __init__(self, agent, cost_map, show_graph=False):

        self.agent = agent

        # cost map there is only
        self.cost_map = cost_map

        self.show_graph = show_graph
        # if self.show_graph:
        #     self.graph = Graph(self.cost_map.shape)
        #     for vertex in self.cost_map.vertices:
        #         self.graph.add_obstacle(vertex)

        # reversed goal and start position
        self.start_node = self.agent.goal
        self.goal_node = self.agent.start

        self.start_node.f_score = calculate_distance(self.agent.start.position,
                                                     self.agent.goal.position,
                                                     DistanceMethod.Manhattan, 1)
        self.start_node.g_score = 0.
        # current node, parent node
        self.agent.experienced_nodes[tuple(self.start_node.position)] = [self.start_node, self.start_node]

        heapq.heappush(self.agent.open_set, (self.start_node.f_score, self.start_node))
        # if self.show_graph:
        #     # print("start position: {}/{}".format(self.start_node.position, self.agent.goal.position))
        #     self.graph.add_open_node(self.start_node.position)
        #     self.graph.show_graph()
        #     # print("obstacles: {}".format(self.graph.obstacles))

    def update_cost_map(self, cost_map):
        self.cost_map = cost_map

    def update_trajectory(self, required_position):
        self.agent.trajectory = []
        if tuple(required_position) in self.agent.experienced_nodes:
            current_node = self.agent.experienced_nodes[tuple(required_position)][0]
            while not self.start_node == current_node:
                self.agent.trajectory.append(current_node)
                current_node = self.agent.experienced_nodes[tuple(current_node.position)][1]
            self.agent.trajectory.append(current_node)
            return True
        else:
            return False

    def run_initial(self):
        success = False
        for test_tuple in self.agent.closed_set:
            if test_tuple[1] == self.goal_node:
                return True

        while not success:
            success = self.step(self.goal_node)
        self.update_trajectory(self.goal_node.position)

    def run_resume(self, required_node):
        success = False
        while len(self.agent.closed_set) > 0:
            current_tuple = heapq.heappop(self.agent.closed_set)
            heapq.heappush(self.agent.open_set, current_tuple)

        while not success:
            success = self.step(required_node)
        self.update_trajectory(self.goal_node.position)
        return success

    def check_node_in_experienced_node(self, node, current_node):
        # check if the node is in open_set
        for test_tuple in self.agent.open_set:
            if test_tuple[1] == node:
                if test_tuple[1].f_score > node.f_score:
                    test_tuple[1].f_score = node.f_score
                    self.agent.experienced_nodes[tuple(node.position)] = [node, current_node]
                    return 1
                else:
                    return 2

        # if the node is a new node or in closed_set
        if tuple(node.position) not in self.agent.experienced_nodes:
            self.agent.experienced_nodes[tuple(node.position)] = [node, current_node]
            heapq.heappush(self.agent.open_set, (node.f_score, node))

            # if self.show_graph:
            #     self.graph.add_open_node(node.position)
            #     self.graph.show_graph()
            return 4
        else:
            return 5

    def step(self, required_node):
        if len(self.agent.open_set) != 0:
            # print("in while loop: {}".format(len(self.agent.open_set)))

            # get current node from open set
            current_tuple = heapq.heappop(self.agent.open_set)
            current_node = current_tuple[1]

            # add current node in closed set
            heapq.heappush(self.agent.closed_set, (current_node.f_score, current_node))
            if current_node == required_node:
                # print("achieved required node")
                return True

            # if self.show_graph:
            #     self.graph.add_closed_node(current_node.position)
            #     self.graph.show_graph()

            # explore neighbors and add them in open set
            neighbors = get_neighbors_reverse(self.cost_map, current_node)
            # print("neighbors: {}".format(neighbors))

            for neighbor in neighbors:

                # calculate f and g
                neighbor.g_score = current_node.g_score + calculate_distance(neighbor.position,
                                                                             current_node.position,
                                                                             DistanceMethod.StraightLine)
                neighbor.f_score = neighbor.g_score + calculate_distance(neighbor.position,
                                                                         self.agent.start.position,
                                                                         DistanceMethod.Manhattan)

                # check if the neighbor is in sets: experienced, open, closed
                self.check_node_in_experienced_node(neighbor, current_node)

                # if self.show_graph:
                #     self.graph.show_graph()

        return False
