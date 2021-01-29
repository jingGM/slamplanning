import math
import numpy as np
import random


class Scenarios(object):
    def __init__(self):
        self.starts = []
        self.goals = []
        self.waypoints = []

        self.collisiondis = 1.
        self.reset()

    def choose_scenarios(self, scenarios_index):
        scenarios = {
            0: self.kitchen,  # empty with fixed start and goal locations
            1: self.empty

        }
        return scenarios[scenarios_index]()

    def reset(self):
        self.starts = []
        self.goals = []
        self.waypoints = []

    def kitchen(self):
        sx, sy, sa = 4, 1., 1.57
        gx, gy, ga = -3, 1., 1.57

        waypoint = []
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)
        return [sx, sy, sa], [gx, gy, ga]

    def empty(self):
        sx, sy, sa = 0, -10., 1.57
        gx, gy, ga = -3, -10., 1.57

        waypoint = []
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)
        return [sx, sy, sa], [gx, gy, ga]
