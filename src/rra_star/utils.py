import numpy as np
from rra_star.node import Node


class DistanceMethod(object):
    StraightLine = object()
    Manhattan = object()


def calculate_distance(position1, position2, method=DistanceMethod.Manhattan, weight=1):
    if method == DistanceMethod.StraightLine:
        return np.sqrt(np.sum((position1 - position2) ** 2)) * weight
    elif method == DistanceMethod.Manhattan:
        return np.sum(abs(position2 - position1)) * weight


class CostMapGrid(object):
    def __init__(self, shape, obstacles):  # obstacles is n*2 matrix
        self.cost_map = np.zeros(tuple(shape), dtype=np.int)
        self.shape = np.array(shape)

        for obs in obstacles:
            self.cost_map[tuple(obs)] = 1

        self.obstacles = obstacles

    def add_obstacle(self, obstacle):
        self.cost_map[tuple(obstacle)] = 1

    def delete_obstacle(self, obstacle):
        self.cost_map[tuple(obstacle)] = 0

    def check_collision(self, position):
        if position[0] >= self.shape[0] or position[0] < 0 or position[1] >= self.shape[1] or position[1] < 0:
            return False
        if self.cost_map[tuple(position)]:
            return True
        else:
            return False

    def check_self_collision(self, position):
        self.delete_obstacle(position)


#
# class CostMap(object):
#     def __init__(self, shape, tolerance=0.5):
#         # shape is a list of tuples
#         self.vertices = []
#         self.shape = np.array(shape)
#
#         if self.shape.shape == (0,):
#             raise Exception("shape need to be defined by width and length")
#         elif self.shape.shape == (2,):
#             self.vertices.append([(-tolerance, -tolerance), (-tolerance, self.shape[1] + tolerance)])
#             self.vertices.append(
#                 [(-tolerance, self.shape[1] + tolerance), (self.shape[0] + tolerance, self.shape[1] + tolerance)])
#             self.vertices.append(
#                 [(self.shape[0] + tolerance, self.shape[1] + tolerance), (self.shape[0] + tolerance, -tolerance)])
#             self.vertices.append([(self.shape[0] + tolerance, -tolerance), (-tolerance, -tolerance)])
#         else:
#             last_vertex = self.shape[-1]
#             for i in range(self.shape.shape[0]):
#                 self.vertices.append([last_vertex, shape[i]])
#                 last_vertex = shape[i]
#
#     def add_obstacle(self, vertices):
#         # the vertices is list of tuples which should clockwisely contour the obstacle
#         if len(vertices) == 2:
#             self.vertices.append([vertices[0], vertices[1]])
#         else:
#             last_vertex = vertices[-1]
#             for i in range(len(vertices)):
#                 self.vertices.append([last_vertex, vertices[i]])
#                 last_vertex = vertices[i]
#
#     def _remove_segment(self, segment):
#         if segment in self.vertices:
#             self.vertices.remove(segment)
#
#     def delete_obstacle(self, vertices):
#         if len(vertices) == 2:
#             segment = [vertices[0], vertices[1]]
#             self._remove_segment(segment)
#         else:
#             last_vertex = vertices[-1]
#             for i in range(len(vertices)):
#                 segment = [last_vertex, vertices[i]]
#                 self._remove_segment(segment)
#                 last_vertex = vertices[i]
#
#     def check_self_collision(self, position, radius):
#         vertices = [[position[0] - radius, position[1] - radius],
#                     [position[0] - radius, position[1] + radius],
#                     [position[0] + radius, position[1] + radius],
#                     [position[0] + radius, position[1] - radius]]
#         self.delete_obstacle(vertices)
#
#     def _vector_multiplication(self, vector1, vector2):
#         return vector1[0] * vector2[1] - vector2[0] * vector1[1]
#
#     def check_collision(self, current_position, next_position):
#         for segment in self.vertices:
#             if min(segment[0][0], segment[1][0]) > max(current_position[0], next_position[0]) or \
#                     max(segment[0][0], segment[1][0]) < min(current_position[0], next_position[0]) or \
#                     min(segment[0][1], segment[1][1]) > max(current_position[1], next_position[1]) or \
#                     max(segment[0][1], segment[1][1]) < min(current_position[1], next_position[1]):
#                 continue
#             else:
#                 ac = np.array([segment[1][0] - current_position[0], segment[1][1] - current_position[1]])
#                 ad = np.array([segment[0][0] - current_position[0], segment[0][1] - current_position[1]])
#                 bc = np.array([segment[1][0] - next_position[0], segment[1][1] - next_position[1]])
#                 bd = np.array([segment[0][0] - next_position[0], segment[0][1] - next_position[1]])
#                 if self._vector_multiplication(ac, ad) * self._vector_multiplication(bc, bd) <= 0 and \
#                         self._vector_multiplication(-1 * ac, -bc) * self._vector_multiplication(-ad, -bd) <= 0:
#                     return True
#         return False


def get_neighbors_forward(cost_map, node):
    neighbors = []
    all_nodes = [np.array([node.position[0] - 1, node.position[1]]),
                 np.array([node.position[0] - 1, node.position[1] - 1]),
                 np.array([node.position[0] - 1, node.position[1] + 1]),
                 np.array([node.position[0] + 1, node.position[1]]),
                 np.array([node.position[0] + 1, node.position[1] - 1]),
                 np.array([node.position[0] + 1, node.position[1] + 1]),
                 np.array([node.position[0], node.position[1] + 1]),
                 np.array([node.position[0], node.position[1] - 1]),
                 # np.array([node.position[0], node.position[1]])
                 ]
    for p in all_nodes:
        if not cost_map.check_collision(p):
            new_node = Node()
            new_node.position = p
            neighbors.append(new_node)
    return neighbors


def get_neighbors_reverse(cost_map, node):
    neighbors = []
    all_nodes = [np.array([node.position[0] - 1, node.position[1]], dtype=np.int16),
                 np.array([node.position[0] - 1, node.position[1] - 1], dtype=np.int16),
                 np.array([node.position[0] - 1, node.position[1] + 1], dtype=np.int16),
                 np.array([node.position[0] + 1, node.position[1]], dtype=np.int16),
                 np.array([node.position[0] + 1, node.position[1] - 1], dtype=np.int16),
                 np.array([node.position[0] + 1, node.position[1] + 1], dtype=np.int16),
                 np.array([node.position[0], node.position[1] + 1], dtype=np.int16),
                 np.array([node.position[0], node.position[1] - 1], dtype=np.int16),
                 # np.array([node.position[0], node.position[1]], dtype=np.int16)
                 ]
    for p in all_nodes:
        if not cost_map.check_collision(p):
            new_node = Node()
            new_node.position = p
            neighbors.append(new_node)
    return neighbors
