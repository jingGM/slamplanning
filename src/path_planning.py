import rospy
import numpy as np
import copy
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header
from slamplanning.srv import SLAMPath, SLAMPathRequest, SLAMPathResponse

from config import RosTopics, PathPlannerConfig, Instruction, PathPlannerType
from rrt_star.main import RRTs
from rra_star.main import RRAs


class PathPlanner:
    def __init__(self, display=True):
        self.display = display
        rospy.init_node("path_planner", anonymous=True)
        self.INSTRUCTIONS = Instruction()

        self.ros_topics = RosTopics()
        self.map_subscriber = rospy.Subscriber(self.ros_topics.map,
                                               OccupancyGrid,
                                               self._map_callback)
        self.odom_subscriber = rospy.Subscriber(self.ros_topics.odom_topic, Odometry, self._odom_callback)
        self.obstacles = None
        self.cost_map = None
        self.map_info = None
        self.map_dimensions = (10, 10)
        self.difference = [0, 0]
        # np.array([(20, 20, 40, 40), (20, 10, 40, 80), (60, 20, 80, 40), (60, 60, 80, 80)])

        self.path_planner = None

        self.instruction_service = rospy.Subscriber(self.ros_topics.initialization,
                                                    Pose,
                                                    self._instruction_callback)

        self.planner_config = PathPlannerConfig()
        self.start = None
        self.start_map = (0, 1)  # starting location
        self.goal = (1,2)  # None
        self.goal_map = (6, 6)  # goal location

        self.origianl_path = []
        self.path = SLAMPathRequest()
        self.path.path.header.seq = 0
        self.path.path.header.frame_id = "path"

        self.plan_tag = True

        # self.publisher = rospy.Publisher(self.ros_topics.path_planning,
        #                                  PoseArray,
        #                                  queue_size=1)
        self.service = rospy.ServiceProxy(self.ros_topics.path_planning, SLAMPath)

    def _instruction_callback(self, pose: Pose):
        self.goal = (pose.position.x, pose.position.y)
        self.plan_tag = True

    def _odom_callback(self, odom: Odometry):
        # if odom.pose.pose.position.x == 0. and odom.pose.pose.position.y == 0.:
        #     return
        # else:
        self.start = (odom.pose.pose.position.x, odom.pose.pose.position.y)

    def _map_callback(self, occupancy_grid: OccupancyGrid):
        obstacles = []
        map = np.array(occupancy_grid.data)
        self.map_info = occupancy_grid.info
        map = np.array(map.reshape((self.map_info.height, self.map_info.width)))
        map = map.T

        x_length = self.map_info.width
        y_length = self.map_info.height

        if self.goal is None or self.start is None:
            return
        self.goal_map = self._convert_pos_to_map(self.goal)
        self.start_map = self._convert_pos_to_map(self.start)
        if self.goal_map[0] < 0:
            self.difference[0] = -self.goal_map[0]
            x_length += self.difference[0]
        if self.goal_map[1] < 0:
            self.difference[1] = -self.goal_map[1]
            y_length += self.difference[1]
        if self.goal_map[0] > x_length:
            x_length = self.goal_map[0]
        if self.goal_map[1] > y_length:
            y_length = self.goal_map[1]

        self.cost_map = np.zeros((x_length, y_length))
        self.map_dimensions = (x_length, y_length)
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i, j] > self.planner_config.threshold:
                    x = i + self.difference[0]
                    y = j + self.difference[1]

                    obs = [x - self.planner_config.robot_radius / self.map_info.resolution,
                           y - self.planner_config.robot_radius / self.map_info.resolution,
                           x + self.planner_config.robot_radius / self.map_info.resolution,
                           y + self.planner_config.robot_radius / self.map_info.resolution]
                    obstacles.append(obs)
                    self.cost_map[x, y] = 1
        self.obstacles = copy.deepcopy(np.array(obstacles))

    def _convert_pos_to_map(self, position):
        while self.map_info is None:
            print("waiting for map topic")
            rospy.sleep(0.1)
        x = (position[0] - self.map_info.origin.position.x) / self.map_info.resolution
        y = (position[1] - self.map_info.origin.position.y) / self.map_info.resolution
        return np.array([x, y], dtype=np.int32)

    def _convert_map_to_pose(self, position):
        while self.map_info is None:
            print("waiting for map topic")
            rospy.sleep(0.1)
        x = (position[0] - self.difference[0]) * self.map_info.resolution + self.map_info.origin.position.x
        y = (position[1] - self.difference[1]) * self.map_info.resolution + self.map_info.origin.position.y
        return np.array([x, y])

    def check_conflict(self):
        tag = False
        for pose in self.origianl_path:
            if pose[0] < self.cost_map.shape[0] or pose[1] < self.cost_map.shape[1]:
                tag |= True
            elif self.cost_map[tuple(pose)]:
                tag |= True
        return tag

    def _path_filter(self, path):
        self.origianl_path.clear()
        if len(path) < 4:
            for i in range(len(path)):
                self.origianl_path.append(path[i])
        else:
            self.origianl_path.append(path[1])
            for i in range(2, len(path)-2, 1):
                first = path[i]
                second = path[i + 1]
                third = path[i + 2]
                if any(second - first != third - second):
                    self.origianl_path.append(first)
            self.origianl_path.append(path[-2])
            self.origianl_path.append(path[-1])

    def plan(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.map_info is None or self.obstacles is None or self.goal is None:
                continue
            if self.check_conflict() or self.plan_tag:
                print("replanning............")
                generated_path = []
                if self.planner_config.path_planner_type == PathPlannerType().rra_star:
                    self.path_planner = RRAs(self.cost_map, self.start_map, self.goal_map)
                    generated_path = self.path_planner.plan()
                elif self.planner_config.path_planner_type == PathPlannerType().rrt_star:
                    self.path_planner = RRTs(np.array([[0, self.map_dimensions[0] + 1], [0, self.map_dimensions[1] + 1]]),
                               self.obstacles,
                               self.start_map, self.goal_map,
                               self.planner_config.length_intersection)
                    generated_path = self.path_planner.plan()

                self._path_filter(generated_path)
                self.path.path.poses.clear()
                for i in range(0, len(self.origianl_path), 1):
                    pose = self._convert_map_to_pose(self.origianl_path[i])
                    position = Pose()
                    position.position.x = pose[0]
                    position.position.y = pose[1]
                    self.path.path.poses.append(position)
                self.plan_tag = False
                print("finished replanning............")

                self.path.path.header.seq += 1
                self.path.path.header.stamp = rospy.get_rostime()
                # self.publisher.publish(self.path)
                success = SLAMPathResponse()
                while not success.success:
                    rospy.wait_for_service(self.ros_topics.path_planning)
                    success = self.service(self.path)
            if self.display and self.path_planner is not None:
                self.path_planner.display(self.origianl_path)

            rate.sleep()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.plan()
