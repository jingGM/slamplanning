import numpy as np


class Node(object):
    def __init__(self, position=np.array([0., 0.])):
        self.position = np.array(position)
        self.dimension = self.position.shape[0]
        self.f_score = 0.  # cost to goal(heuristic) + cost to come
        self.g_score = 0.  # cost to come

    def __eq__(self, other):
        judge = True
        for i in range(self.dimension):
            judge &= self.position[i] == other.position[i]
        return judge

    def __le__(self, other):
        return self.f_score <= other.f_score

    def __lt__(self, other):
        return self.f_score < other.f_score

    def __ge__(self, other):
        return self.f_score >= other.f_score

    def __gt__(self, other):
        return self.f_score > other.f_score
