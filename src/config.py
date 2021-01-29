import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid


class Instruction:
    def __init__(self):
        self.start = "start"
        self.goal = "goal"


class PathPlannerType:
    def __init__(self):
        self.rra_star = "rra_star"
        self.rrt_star = "rrt_star"


class RosTopics:
    def __init__(self):
        self.map = "/map"
        self.initialization = "/initialization"
        self.path_planning = "/total_path"

        self.states_topic = "/gazebo/model_states"
        self.set_service = '/gazebo/set_model_state'

        # self.scan_topic = '/scan'
        # self.odom_topic = "/jackal_velocity_controller/odom"
        # self.action_topic = "/jackal_velocity_controller/cmd_vel"
        # self.robot_name = "jackal"

        self.scan_topic = "/scan"
        self.action_topic = "/cmd_vel"
        self.odom_topic = "/odom"
        self.robot_name = "turtlebot3"


class PathPlannerConfig:
    def __init__(self, robot_radius=0.2, occupancy_threshold=60):
        self.robot_radius = robot_radius
        self.threshold = occupancy_threshold
        self.path_planner_type = PathPlannerType().rra_star

        # For RRT Star
        self.length_edge = np.array([(8, 4)])  # length of tree edges
        self.length_intersection = 0.01  # length of smallest edge to check for intersection with obstacles
        self.max_samples = 1024  # max number of samples to take before timing out
        self.rewire_count = 32  # optional, number of nearby branches to rewire
        self.prob_goal = 0.1  # probability of checking for a connection to goal

        # For RRA Star
