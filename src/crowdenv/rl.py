if __name__ == "__main__":
    import sys
    sys.path.append("../")

import rospy
import math
import copy
import threading
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from crowdenv.networks import NNModule
from crowdenv.scenarios import Scenarios


class TOPICS(object):
    def __init__(self):
        self.scan_topic = 'jackal0/scan_filtered'
        self.reset_service = '/gazebo/reset_world'
        self.set_service = '/gazebo/set_model_state'
        self.states_topic = "/gazebo/model_states"

        self.odom_topic = "jackal0/jackal_velocity_controller/odom"
        self.action_topic = "jackal0/jackal_velocity_controller/cmd_vel"


class CrowdENV:
    def __init__(self,
                 scenarios_index=0,
                 collision_threshold=0.22,
                 target_threshold=0.8,
                 step_time=0.1,
                 max_steps=1000,
                 random_seed=1):

        self.seed = random_seed
        self.robot_name = "turtlebot3"
        self.collision_threshold = collision_threshold
        self.target_threshold = target_threshold
        self.start = None
        self.goal = None
        self.step_time = step_time
        self.max_steps = max_steps

        self.vel_threshold = ((0., 1.), (-1., 1.))
        self.lidar_threshold = 4.

        self.scenarios = Scenarios()
        self.scenarios_index = scenarios_index

        self.topics = TOPICS()

        self.trajectory = []

        self.position = None
        self.velocity = None
        self.action = None
        self.reward = None
        self.scan = None
        self.scan_single = None
        self.rel_goal = None

        self.terminatec = False
        self.terminateg = False
        self.terminates = False

        self.last_time = rospy.get_rostime().to_sec()
        self.step_size = 0

        self._generate_advertisements()

        self.reset()

    def reset(self, scenarios_index=None):
        if scenarios_index is not None:
            self.scenarios_index = scenarios_index
        self.start, self.goal = self.scenarios.choose_scenarios(self.scenarios_index)
        self._set_robot_srv()

        self.position = None
        self.velocity = None
        self.action = None
        self.reward = None
        self.scan = None
        self.rel_goal = None
        self.last_rel_goal = None

        self.terminatec = False
        self.terminateg = False
        self.terminates = False

        self.last_time = rospy.get_rostime().to_sec()
        self.step_size = 0

        while (self.position is None) or (self.scan is None) or (self.velocity is None):
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

        self.pose_subscriber = rospy.Subscriber(self.topics.states_topic, ModelStates, self._state_callback,
                                                queue_size=1)

    def _scan_callback(self, data):
        lidar_data = np.flip(data.ranges)
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

    def _odom_callback(self, data):
        self.velocity = np.array([data.twist.twist.linear.x, data.twist.twist.angular.z])

    def _state_callback(self, states):
        for i in range(len(states.name)):
            if states.name[i] == self.robot_name:
                self.position = states.pose[i]

    def _calculate_rel_goal(self):
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
        if self.step_size >= self.max_steps:
            self.terminates = True

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

    def get_terminate(self):
        return self.terminatec or self.terminateg or self.terminates

    def get_observation(self):
        processed_lidar = copy.deepcopy(np.array(self.scan))
        processed_vel = self.velocity
        processed_goal = self._calculate_rel_goal()
        obs = [processed_lidar, processed_goal, processed_vel]
        # done = self.get_terminate()
        return obs, [self.terminatec, self.terminateg, self.terminates], [self.position.position.x,
                                                                          self.position.position.y]


class TrajectoryGenerator:
    def __init__(self, iterations=50):
        self.env = CrowdENV(scenarios_index=10, max_steps=1000)
        self.policy = NNModule(velocity_threshold=self.env.vel_threshold, path="./")
        self.iterations = iterations

    def run(self, scenarios=range(0, 20, 1)):
        for i in scenarios:
            self.get_trajectory(i)

    def get_trajectory(self, scenario):
        # for i in range(steps):
        value = 0
        total_trajectories = []

        obs, done, position = self.env.reset(scenario)
        last_position = position
        length = 0
        length_list = []

        time_list = []
        last_time = rospy.get_rostime().to_sec()
        successes = 0
        while (not rospy.is_shutdown()) and (value < self.iterations):
            print("{}, {}, {}, {}".format(obs[0].shape, obs[1], obs[2], done))
            if done[0] or done[1] or done[2]:
                if done[1] and not done[0] and not done[2]:
                    successes += 1
                    length_list.append(length)
                    time_list.append(rospy.get_rostime().to_sec()-last_time)
                length = 0
                last_time = rospy.get_rostime().to_sec()
                value += 1
                total_trajectories.append(copy.deepcopy(self.env.trajectory))
                print(done, end=" ")
                obs, done, position = self.env.reset(scenario)

            action = self.policy.predict(obs)
            obs, done, position = self.env.step(action)
            length += math.sqrt((position[0] - last_position[0])**2 + (position[1] - last_position[1])**2)
            last_position = copy.deepcopy(position)
        # np.save("trajectory_{}".format(scenario), np.asarray(total_trajectories))
        print("length: {}, time: {}, succussR: {}".format(np.mean(length_list),
                                                          np.mean(time_list),
                                                          successes))


def run_iterations():
    tg = TrajectoryGenerator(10)
    tg.run([6])


if __name__ == "__main__":
    rospy.init_node('robot', anonymous=True)

    t1 = threading.Thread(target=rospy.spin)
    t2 = threading.Thread(target=run_iterations)

    t1.start()
    t2.start()

    t2.join()
    t1.join()
