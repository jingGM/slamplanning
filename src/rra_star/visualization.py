import numpy as np
import cv2
import rospy


class Graph(object):
    def __init__(self, shape, margin=5):
        self.augmentation = 4
        self.margin = margin
        self.shape = shape * self.augmentation
        self.background = np.ones((self.shape, self.shape, 3), dtype=np.uint8) * 255
        # self.obstacles = None
        # self.starts = None
        # self.goals = None
        # self.trajectories = None

    def show_obstacles(self, obstacles):
        # self.obstacles = obstacles
        for point in obstacles:
            cv2.circle(self.background, (int(point[0] * self.augmentation), int(point[1] * self.augmentation)), 5, (255, 0, 0), -1, 0)

    def show_map(self, obstacles, starts, goals, trajectories, tag=False):
        # while not rospy.is_shutdown():
            if len(trajectories) != 0:
                for i in range(0, np.array(trajectories).shape[1] - 1, 1):
                    for rob in range(np.asarray(trajectories).shape[0]):
                        cv2.line(self.background,
                                 (int(trajectories[rob][i][0] * self.augmentation), int(trajectories[rob][i][1] * self.augmentation)),
                                 (int(trajectories[rob][i + 1][0] * self.augmentation), int(trajectories[rob][i + 1][1] * self.augmentation)),
                                 (0, 0, 255), 5, -1, 0)
            self.show_obstacles(obstacles)
            self.show_agent(starts, (0, 255, 255))
            self.show_agent(goals, (0, 255, 0))

            cv2.flip(self.background, 0, self.background)
            # cv2.rotate(self.background, cv2.ROTATE_90_COUNTERCLOCKWISE, self.background)
            cv2.imshow("selfView", self.background)
            cv2.waitKey(1000)
            self.background = np.ones((self.shape, self.shape, 3), dtype=np.uint8) * 255
            # if tag:
            #     break

    def show_path(self, paths):
        for edge in paths:
            last_point = None
            for point in edge:
                if last_point is not None:
                    cv2.line(self.background, last_point, (int(point[0] * self.augmentation), int(point[1] * self.augmentation)),
                             (0, 0, 255), 5, -1, 0)
                last_point = (int(point[0]), int(point[1]))

    def show_agent(self, points, color=(0, 255, 255)):
        for point in points:
            cv2.circle(self.background, (int(point[0] * self.augmentation), int(point[1] * self.augmentation)), 5, color, -1, 0)
