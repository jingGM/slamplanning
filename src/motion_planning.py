import rospy
import math
import copy
import threading
import numpy as np

from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from slamplanning.srv import SLAMPath, SLAMPathRequest, SLAMPathResponse

from crowdenv.networks import NNModule

from config import RosTopics


class MotionPlanner:
    def __init__(self,
                 collision_threshold=0.22,
                 target_threshold=0.8,
                 step_time=0.1
                 ):
        rospy.init_node("motion_planner", anonymous=True)
        self.start = np.array([0., -5, 0.])
        self.robot_name = "turtlebot3"
        self.vel_threshold = ((0., 1.), (-1., 1.))
        self.step_time = step_time
        self.collision_threshold = collision_threshold
        self.target_threshold = target_threshold
        self.topics = RosTopics()

        self.lidar_threshold = 4.
        self.waypoint_threshold = 1.

        self.policy = NNModule(velocity_threshold=self.vel_threshold, path="./crowdenv/")

        self.trajectory = []
        self.path = []

        self.position = None
        self.velocity = None
        self.action = None
        self.reward = None
        self.scan = None
        self.scan_single = None
        self.rel_goal = None
        self.goal = None

        self.terminatec = False
        self.terminateg = False

        self.last_time = rospy.get_rostime().to_sec()
        self.step_size = 0

        self._generate_advertisements()

        self.reset()

    def reset(self):
        self._set_robot_srv()

        self.position = None
        self.velocity = None
        self.action = None
        self.reward = None
        self.scan = None
        self.rel_goal = None
        self.goal = [10, -5]

        self.terminatec = False
        self.terminateg = False

        self.last_time = rospy.get_rostime().to_sec()
        self.step_size = 0

        while (self.position is None) or (self.scan is None) or (self.velocity is None) or (self.goal is None):
            pass
        return self.get_observation()

    def _set_robot_srv(self):
        state_msg = ModelState()
        state_msg.model_name = self.robot_name
        state_msg.pose.position.x = self.start[0]
        state_msg.pose.position.y = self.start[1]
        state_msg.pose.orientation.z = np.sin(self.start[2] / 2)
        state_msg.pose.orientation.w = np.cos(self.start[2] / 2)

        rospy.wait_for_service(self.topics.set_service)
        try:
            set_robot = rospy.ServiceProxy(self.topics.set_service, SetModelState)
            set_robot(state_msg)
        except rospy.ServiceException as err:
            print("Service call failed: {}".format(err))
        rospy.sleep(1)

    def _generate_advertisements(self):
        self.scan_subscriber = rospy.Subscriber(self.topics.scan_topic, LaserScan, self._scan_callback, queue_size=1)

        self.publisher = rospy.Publisher(self.topics.action_topic, Twist, queue_size=10)

        self.odom_subscriber = rospy.Subscriber(self.topics.odom_topic, Odometry, self._odom_callback, queue_size=1)

        # self.path_subscriber = rospy.Subscriber(self.topics.path_planning, PoseArray, self._path_callback, queue_size=1)
        self.path_service = rospy.Service(self.topics.path_planning, SLAMPath, self._path_callback)

    def _path_callback(self, data: PoseArray):
        # self.path.clear()
        path = []
        for pose in data.poses:
            # pose = Pose()
            path.append([pose.position.x, pose.position.y])
        self.path = copy.deepcopy(path)

        success = SLAMPathResponse()
        success.success = True
        return success

    def _scan_callback(self, data):
        lidar_data = np.flip(data.ranges)
        lidar_data = np.where(np.isinf(lidar_data), self.lidar_threshold, lidar_data)
        lidar_data = np.where(np.isnan(lidar_data), self.lidar_threshold, lidar_data)
        lidar_data = np.where(lidar_data < 0., 0., lidar_data)

        scan_single = np.array([lidar_data]).transpose() / self.lidar_threshold
        if self.scan is None:
            self.scan = np.concatenate((copy.deepcopy(scan_single),
                                        copy.deepcopy(scan_single),
                                        copy.deepcopy(scan_single)), axis=1)
        else:
            self.scan = np.concatenate((self.scan[:, 1:], scan_single), axis=1)
        min_dis_collision = lidar_data.min()
        if min_dis_collision < self.collision_threshold:
            self.terminatec = True
            self.publisher.publish(Twist())

    def _odom_callback(self, data: Odometry):
        self.position = data.pose.pose
        self.velocity = np.array([data.twist.twist.linear.x, data.twist.twist.angular.z])

    def _extract_nearest_goal(self):
        distance = 0
        while distance < self.waypoint_threshold and len(self.path) > 0:
            goal = self.path.pop(0)
            distance = math.sqrt(abs(self.position.position.x - goal[0])**2 + abs(self.position.position.y - goal[1])**2)
            self.goal = copy.deepcopy(goal)

    def _calculate_rel_goal(self):
        self._extract_nearest_goal()
        robot_rel_y = self.goal[1] - self.position.position.y
        robot_rel_x = self.goal[0] - self.position.position.x
        distance = math.hypot(robot_rel_x, robot_rel_y)

        sin_y = 2 * (self.position.orientation.w * self.position.orientation.z +
                     self.position.orientation.x * self.position.orientation.y)
        cos_y = 1 - 2 * (self.position.orientation.y ** 2 + self.position.orientation.z ** 2)
        orientation = np.arctan2(robot_rel_y, robot_rel_x) - np.arctan2(sin_y, cos_y)
        if orientation > np.pi:
            orientation -= 2 * np.pi
        elif orientation < -np.pi:
            orientation += 2 * np.pi
        else:
            orientation = orientation

        if distance <= self.target_threshold:
            self.terminateg = True
        return np.array([distance, orientation])

    def step(self, action):
        velocity = Twist()
        velocity.linear.x = action[0]
        velocity.angular.z = action[1]

        while not (self.publisher.get_num_connections() > 0):
            pass
        self.publisher.publish(velocity)
        self.step_size += 1

        while rospy.get_rostime().to_sec() - self.last_time < self.step_time:
            pass
        self.last_time = rospy.get_rostime().to_sec()

        obs, done, position = self.get_observation()

        self.trajectory.append([obs,
                                action,
                                self.step_size,
                                self.position,
                                rospy.get_rostime(),
                                (self.terminatec, self.terminateg)])
        return obs, done, position

    def run(self):
        obs, done, position = self.reset()
        while not rospy.is_shutdown():
            action = self.policy.predict(obs)
            if done[1]:
                action = np.array([0., 0.])
            elif done[0]:
                print("collided")
                action = np.array([-0.2, 0.])
            obs, done, position = self.step(action)

    def get_terminate(self):
        return self.terminatec or self.terminateg

    def get_observation(self):
        processed_lidar = copy.deepcopy(np.array(self.scan))
        processed_vel = self.velocity
        processed_goal = self._calculate_rel_goal()
        obs = [processed_lidar, processed_goal, processed_vel]
        # done = self.get_terminate()
        return obs, [self.terminatec, self.terminateg], [self.position.position.x, self.position.position.y]


if __name__ == "__main__":
    motionplanner = MotionPlanner()
    motionplanner.run()