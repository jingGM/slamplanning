import math
import numpy as np
import random
from crowdenv.simulator.utils.types import ScenarioTypes


class Scenarios(object):
    def __init__(self):
        self.starts = []
        self.goals = []
        self.waypoints = []

        self.collisiondis = 1.
        self.reset()

    def choose_scenarios(self, scenarios_index):
        scenarios = {
            0: self.empty_r,  # empty with fixed start and goal locations
            1: self.empty_l,  # empty with fixed start and goal locations
            2: self.random_empty_r,  # empty world with random start and goal locations
            3: self.random_empty_l,  # empty world with random start and goal locations
            4: self.corridor_static_l,  # corridor scenario with static obstacle, robot is in left side
            5: self.corridor_static_r,  # corridor scenario with static obstacle, robot is in right side
            6: self.dynamic_sides,
            7: self.narrow_ped,
            8: self.narrow_ped,  # corridor scenario with people walking to robot
            9: self.empty_straight,  # empty scenarios
            10: self.narrow_ped_2peds,  # corridor scenario with 2 people walking to robot
            11: self.narrow_ped_6m,  # corridor scenario with 2 people walking to robot

        }
        return scenarios[scenarios_index]()

    def reset(self):
        self.starts = []
        self.goals = []
        self.waypoints = []

    def dynamic_sides(self):
        sx, sy, sa = 3, -4., 1.57
        gx, gy, ga = 3, 4., 1.57

        # if self.type == ScenarioTypes.TOTAL:
        #     sx, sy = sx + 20., sy + 20.
        #     gx, gy = gx + 20., gy + 20.

        waypoint = []
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)
        return [sx, sy, sa], [gx, gy, ga]

    def corridor_static_l(self):
        sx, sy, sa = -2., -3., 1.57
        gx, gy, ga = -2.,  3., 1.57

        # if self.type == ScenarioTypes.TOTAL:
        #     sx, sy = sx + 20., sy + 20.
        #     gx, gy = gx + 20., gy + 20.

        waypoint = [[7, 0], [gx, gy]]
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)
        return [sx, sy, sa], [gx, gy, ga]

    def corridor_static_r(self):
        sx, sy, sa = 2., -3., 1.57
        gx, gy, ga = 2.,  3., 1.57

        # if self.type == ScenarioTypes.TOTAL:
        #     sx, sy = sx + 20., sy + 20.
        #     gx, gy = gx + 20., gy + 20.

        waypoint = [[13., 0], [gx, gy]]
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)
        return [sx, sy, sa], [gx, gy, ga]

    def narrow_ped(self):
        sx, sy, sa = 11, 0., 3.14
        gx, gy, ga = 0, 0, 3.14
        waypoint = [[gx, gy]]
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)
        start = [sx, sy, sa]
        goal = [gx, gy, ga]
        return start, goal

    def narrow_ped_6m(self):
        sx, sy, sa = 11, 0., 3.14
        gx, gy, ga = -11.0, 0.5, 3.14
        waypoint = [[gx, gy]]
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)

        return [sx, sy, sa], [gx, gy, ga]

    def narrow_ped_2peds(self):
        sx, sy, sa = 11, 0., 3.14
        gx, gy, ga = -10.0, 0., 3.14
        waypoint = [[gx, gy]]
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)

        return [sx, sy, sa], [gx, gy, ga]

    def empty_straight(self):
        sx, sy, sa = 0., 0., 0.
        if self.type == ScenarioTypes.TOTAL:
            sx, sy = sx, sy - 10.
        else:
            sx, sy = sx, sy - 10.

        gx, gy, ga = sx + 4., sy + 0., 0.
        waypoint = []
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)

        return [sx, sy, sa], [gx, gy, ga]

    def empty_r(self):
        sx, sy, sa = 0., 0., 0.
        if self.type == ScenarioTypes.TOTAL:
            sx, sy = sx, sy - 10.
        else:
            sx, sy = sx, sy - 10.

        gx, gy, ga = sx + 4., sy + 4., 0.
        waypoint = [[sx + 1., sy + 1.],
                    [sx + 2., sy + 2.],
                    [sx + 3., sy + 3.]]
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)

        return [sx, sy, sa], [gx, gy, ga]

    def empty_l(self):
        sx, sy, sa = -10., 0., 0.
        if self.type == ScenarioTypes.TOTAL:
            sx, sy = sx, sy - 10.

        gx, gy, ga = sx + 4., sy - 4., 0.
        waypoint = [[sx + 1., sy - 1.],
                    [sx + 2., sy - 2.],
                    [sx + 3., sy - 3.]]
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)

        return [sx, sy, sa], [gx, gy, ga]

    def random_empty_r(self):
        sx, sy, sa = 1., 1., random.uniform(0., 1.) * 6.28 - 3.14
        gx, gy, ga = 5. * random.uniform(0., 1.), 5 * random.uniform(0., 1.), 0.
        if self.type == ScenarioTypes.TOTAL:
            sx, sy = sx + 25., sy - 10.
            gx, gy = gx + 25., gy - 10.
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        return [sx, sy, sa], [gx, gy, ga]

    def random_empty_l(self):
        sx, sy, sa = 2., 2., random.uniform(0., 1.) * 6.28 - 3.14
        gx, gy, ga = 5. * random.uniform(0., 1.), 5. * random.uniform(0., 1.), 0.
        if self.type == ScenarioTypes.TOTAL:
            sx, sy = sx + 15., sy - 10.
            gx, gy = gx + 15., gy - 10.

        waypoint = []
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        return [sx, sy, sa], [gx, gy, ga]

    def test_location(self, x, y, obstacles):
        success = True
        for obs in obstacles:
            # obs: x,y,w,h
            left = obs[0] - obs[2] / 2. - self.collisiondis
            right = obs[0] + obs[2] / 2. + self.collisiondis
            up = obs[1] + obs[3] / 2. + self.collisiondis
            down = obs[1] - obs[3] / 2. - self.collisiondis
            if right > x > left and up > y > down:
                success = False
        return success

    def generate_position(self, obstacles):
        environ_size = 5.
        success_set = False
        x, y = 0.0, 0.0
        while not success_set:
            x = random.random() * environ_size
            y = random.random() * environ_size
            success_set = self.test_location(x, y, obstacles)
        a = 2. * np.pi * random.uniform(0., 1.) - np.pi
        return [x, y, a]

    def random_environment_l(self):
        obstacles = [[2., 0., 1., 0.5],
                     [5., 0., 0.5, 1.],
                     [1., 2., 0.5, 1.],
                     [1., 4., 1., 0.5],
                     [3., 3., 1., 0.5],
                     [4., 2., 1., 0.5],
                     [4., 5., 0.5, 1.]]
        sx, sy, sa = self.generate_position(obstacles)
        obstacles.append([sx, sy, self.collisiondis, self.collisiondis])
        gx, gy, ga = self.generate_position(obstacles)
        if self.type == ScenarioTypes.TOTAL:
            sx, sy = sx + 15., sy
            gx, gy = gx + 15., gy

        waypoint = []
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)
        print(sx, sy, gx, gy)

        return [sx, sy, sa], [gx, gy, ga]

    def random_environment_r(self):
        obstacles = [[5., 5., 1., 1.],
                     [4., 1., 1., 1.],
                     [3., 3., 1., 1.],
                     [1., 1., 1., 0.5],
                     [1., 4., 0.5, 1.]]
        sx, sy, sa = self.generate_position(obstacles)
        obstacles.append([sx, sy, self.collisiondis, self.collisiondis])
        gx, gy, ga = self.generate_position(obstacles)
        if self.type == ScenarioTypes.TOTAL:
            sx, sy = sx + 25., sy
            gx, gy = gx + 25., gy

        waypoint = []
        self.starts.append([sx, sy, sa])
        self.goals.append([gx, gy, ga])
        self.waypoints.append(waypoint)
        print(sx, sy, gx, gy)

        return [sx, sy, sa], [gx, gy, ga]
