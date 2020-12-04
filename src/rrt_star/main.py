import numpy as np
from rrt_star.rrt.rrt_star import RRTStar
from rrt_star.search_space.search_space import SearchSpace
from rrt_star.utilities.plotting import Plot


class RRTs:
    def __init__(self,
                 dimensions,
                 obstacles,
                 start, goal,
                 length_intersection,
                 length_edge=np.array([(8, 4)]),
                 max_samples=1024,
                 prob_goal=0.1,
                 rewire_count=32,
                 display=True):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.map = obstacles
        self.space = SearchSpace(dimensions, self.map)
        self.rrt = RRTStar(self.space,
                           length_edge,
                           self.start, self.goal,
                           max_samples,
                           length_intersection,
                           prob_goal,
                           rewire_count)
        self.path = [goal]

    def plan(self):
        self.path = self.rrt.rrt_star()
        return self.path

    def display(self, path=None):
        if path is not None:
            self.path = path
        plot = Plot("rrt_star_2d")
        plot.plot_tree(self.space, self.rrt.trees)
        plot.plot_path(self.space, self.path)
        plot.plot_obstacles(self.space, self.map)
        plot.plot_start(self.space, self.start)
        plot.plot_goal(self.space, self.goal)
        plot.draw(auto_open=True)

