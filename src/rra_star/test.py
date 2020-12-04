import numpy as np
from rra_star.utils import CostMapGrid
from rra_star.node import Node
from rra_star.agent import Agent
from rra_star.RRAStar import RRAStar
import random
import copy
from rra_star.visualization import Graph


def generate_obstacles(shape, number, constraints=[]):
    index_range = shape - 1
    random.seed(1)
    obstacles = copy.deepcopy(constraints)
    positions = []
    for i in range(number):
        x: int
        y: int
        available = False
        while not available:
            x = round(index_range * random.random())
            y = round(index_range * random.random())
            available = True
            for obs in obstacles:
                if obs[0] == x and obs[1] == y:
                    available = False
        positions.append([x, y])
        obstacles.append([x, y])
    return positions


def test_whca_grid():
    shape = 32
    display = Graph(shape)
    obstacles = generate_obstacles(shape, 50)

    # print("trjactory: {}".format(agent.experienced_nodes))

    start = Node([0., 0.])
    goal = Node([20., 30.])
    agent = Agent(start, goal)
    cost_map = CostMapGrid((shape,shape), obstacles)

    algorithm = RRAStar(agent, cost_map, show_graph=True)
    success = algorithm.run_initial()


    trajectories = []
    path = []
    for pos in agent.trajectory:
        path.append(pos.position)
        print("{}".format(pos.position), end="; / ")
    trajectories.append(copy.deepcopy(path))
    print(" ")

    display.show_map(obstacles, [start.position], [goal.position], trajectories)


if __name__ == "__main__":
    print("start testing")
    # test_RRAStar()
    test_whca_grid()
