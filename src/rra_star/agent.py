import numpy as np
from rra_star.node import Node
import heapq


class Agent(object):
    def __init__(self, start: Node, goal: Node):
        self.start = start
        self.goal = goal

        self.open_set = []
        # heapq.heappush(self.open_set, (self.start.f_score, self.start))

        self.closed_set = []

        self.experienced_nodes = {}  # format: {tuple(node.position): node, next_node}

        self.trajectory = [self.start]  # format: nodes

        # self.last_position = None

    def clear_info(self):
        self.open_set = []
        self.closed_set = []
        self.experienced_nodes = []