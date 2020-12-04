from rra_star.utils import CostMapGrid
from rra_star.node import Node
from rra_star.agent import Agent
from rra_star.visualization import Graph
from rra_star.RRAStar import RRAStar
import random
import copy
import numpy as np


class RRAs:
    def __init__(self, cost_map, start, goal):
        self.start = Node(start)
        self.goal = Node(goal)
        self.agent = Agent(self.start, self.goal)

        obstacles = np.array(np.where(np.asarray(cost_map) == 1))
        self.obstacles = obstacles.T
        self.cost_map = CostMapGrid(np.asarray(cost_map).shape, self.obstacles)

        self.graph = Graph(max(self.cost_map.shape))
        # self.graph.show_map(self.obstacles, [self.start.position], [self.goal.position], [])
        self.algorithm = RRAStar(self.agent, self.cost_map)

        self.path = []

    def plan(self):
        success = self.algorithm.run_initial()
        self.path.clear()
        for pos in self.agent.trajectory:
            self.path.append(pos.position)
        return np.array(self.path)

    def display(self, path=None):
        if path is not None:
            self.path = path
        self.graph.show_map(self.obstacles, [self.start.position], [self.goal.position], [self.path])