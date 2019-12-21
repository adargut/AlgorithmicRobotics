import random
from fractions import Fraction
from typing import List

from arr2_epec_seg_ex import *
from collision_detection import *
from math import sqrt
from collections import defaultdict, namedtuple
import heapq


class PRM(object):
    """
    sampling strategy = uniform?
    * collision detection = given
    distance metric =
    * nearest neighbor search = kd tree
    connection strategy = lazy evalutaion?
    local planner
    query phase = A*?
    :return:
    """

    def __init__(self, rod_length, obstacles: List[Polygon_2], milestones_count, epsilon):
        self.rod_length = rod_length
        self.obstacles = obstacles
        self.milestones_count = milestones_count
        self.epsilon = epsilon

        self.kd_tree = Kd_tree()
        self.roadmap = Graph()  # FIXME: Implement Graph

    # Check if uniformly sampled point is legal
    def _is_point_free(self, point: Point_3, l):
        return is_position_valid(point.x(), point.y(), point.z(), self.rod_length, self.obstacles, self.epsilon)

    # Find nearest neighbors of sampled point, using KD-trees
    def _find_k_nearest_neighbors(self, query_point: Point_3, k, metric_function):
        eps = 1  # TODO maybe change epsilon?
        search_nearest = True
        sort_neighbors = True

        search = K_neighbor_search(
            self.kd_tree, query_point, k, eps, search_nearest, metric_function(), sort_neighbors)

        lst = []
        return search.k_neighbors(lst)

    # Check if edge can be added between two points in roadmap
    def _is_edge_legal(self, source: Point_3, destination: Point_3, sampling_distance=0.1):
        edge = Line_2(source, destination)
        # TODO: Use proper metric
        length = sqrt(squared_distance(source, destination))
        samples_amount = round(length * sampling_distance)
        s_x, s_y, s_z = point_3_to_xyz(source)
        for t in range(0, samples_amount):
            current_point = xyz_to_point_3(
                s_x + t * sampling_distance, s_y + t * sampling_distance, s_z + t * sampling_distance)
            if not self._is_point_free(current_point):
                return False
        return True

    def calculate_roadmap(self):
        for milestone in range(self.milestones_count):
            # Get a random valid position
            is_free_sample = False
            while not is_free_sample:
                curr_sample = generate_random_point()
                is_free_sample = self._is_point_free(curr_sample)
            self.add_to_roadmap(curr_sample)

    def add_to_roadmap(self, point, threshold=50):
        # Connect to the nearest neighbors
        # TODO: Maybe concider connected components efficiency?
        nearest_neighbors = self._find_k_nearest_neighbors(
            point, Euclidian_distance)
        # Locality test
        nearest_node, dist = nearest_neighbors[0]
        if dist > threshold:
            pass

        sample_is_milestone = False
        for neighbor in nearest_neighbors:
            if self._is_edge_legal(point, neighbor):
                sample_is_milestone |= True
                weight = metric(neighbor, point)  # TODO: use metric
                self.roadmap.add_edge(neighbor, point, weight)
        self.graph.add_vertex(point)
        self.kd_tree.insert(point)


class Graph(object):
    def __init__(self):
        self.graph = defaultdict(list)

    def add_edge(self, source, dest, weight):
        self.graph[source].append(((source, dest), weight))
        self.graph[dest].append(((dest, source), weight))

    def add_vertex(self, v):
        self.graph[v] = []

    @property
    def vertices(self):
        return self.graph.keys()

    @property
    def edges(self):
        return self.graph.values()

    def is_exists(self, node):
        return node in self.vertices

    def neighbors(self, v):
        if self.is_exists(v):
            return self.graph[v]

    def djikstra(self, source) -> dict:
        distances = {n: float('infinity') for n in self.vertices()}
        distances[source] = 0
        pq = [(0, source)]
        path = {}

        while len(pq) > 0:
            current_weight, min_node = heapq.heappop(pq)

            if current_weight > distances[min_node]:
                continue

            neighbors = self.neighbors(min_node)
            for edge, edge_weight in neighbors:
                neighbor = edge[1]
                weight = current_weight + edge_weight
                if weight < distances[neighbor]:
                    distances[neighbor] = weight
                    heapq.heappush(pq, (weight, neighbor))
                    path[neighbor] = min_node

        return path

    def shortest_path(self, source, dest):
        fathers = self.djikstra(source)
        if fathers == {}:
            return []

        path = [source]
        curr_node = dest

        while curr_node != source:
            path.append(curr_node)
            if curr_node not in fathers:
                return []  # a free path does not exist
            curr_node = fathers[curr_node]
        path.append(dest)
        return path


############### Utilities ###############

# Return a uniformly sampled point in R2 x S1
def generate_random_point():
    # TODO find real bounds
    minX, maxX, minY, maxY = -5000, 5000, -5000, 5000
    rand_x_coord = random.uniform(minX, maxX)
    rand_y_coord = random.uniform(minY, maxY)
    rand_theta_coord = Fraction(int(round(16 * random.uniform(0, 22 / 7))), 16)
    return Point_3(rand_x_coord, rand_y_coord, float(rand_theta_coord))


def point_3_to_xyz(p):
    return (p.x().to_double(), p.y().to_double(), p.z().double())


def xyz_to_point_3(x, y, z):
    return Point_3(x, y, z)


def handle_clockwise(path):
    pass  # FIXME

##########################################


def generate_path(path, length, obstacles, origin, destination):
    path.append((FT(Gmpq(230)), FT(Gmpq(500)), FT(Gmpq("0/3")), True))
    path.append((FT(Gmpq(300)), FT(Gmpq(1000)), FT(Gmpq("2/1")), True))
    path.append((FT(Gmpq(230)), FT(Gmpq(700)), FT(Gmpq("1/5")), False))
    return path


def run_algorithm(rod_length, obstacles: List[Polygon_2], milestones_count, epsilon, source, dest):
    prm = PRM(rod_length, obstacles, milestones_count, epsilon)
    prm.calculate_roadmap()

    prm.add_to_roadmap(source, threshold=0)
    prm.add_to_roadmap(dest, threshold=0)

    path = prm.roadmap.shortest_path(source, dest)
    print(path)
    path = handle_clockwise(path)

    return path
