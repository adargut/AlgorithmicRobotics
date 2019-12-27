import random
import sys
import numpy as np
from fractions import Fraction
from typing import List
from conversions import *
from collision_detection import *
from math import sqrt
from collections import defaultdict
import heapq

PI = FT(22 / 7)
NEIGHBORS = 5
PLUS_INF = FT(sys.maxsize)
MINUS_INF = FT((-1) * (sys.maxsize - 1))


## --------------- PRM  ------------------

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

    def __init__(self, rod_length, obstacles: List[Polygon_2], milestones_count, epsilon, metric):

        self.rod_length = rod_length
        self.obstacles = obstacles
        self.milestones_count = milestones_count
        self.epsilon = epsilon
        self.metric = metric
        self.kd_tree = Kd_tree()
        self.roadmap = RoadmapGraph()
        self.bounds = []

    def _is_point_free(self, point: Point_3):

        return is_position_valid(point.x(), point.y(), point.z(), self.rod_length, self.obstacles, self.epsilon)

    def compute_bounds(self):

        minX, maxX, minY, maxY = PLUS_INF, MINUS_INF, PLUS_INF, MINUS_INF

        for obstacle in self.obstacles:
            for vertex in tuples_list_to_polygon_2(obstacle).vertices():
                x, y = vertex.x(), vertex.y()
                minX = min(minX, x)
                minY = min(minY, y)
                maxX = max(maxX, x)
                maxY = max(maxY, y)

        self.bounds = [minX, maxX, minY, maxY]

    def _find_k_nearest_neighbors(self, query_point: Point_3, k):

        print("## SEARCHING FOR NEAREST NEIGHBORS ##")
        eps = FT(Gmpq(0.0))  # TODO maybe change epsilon?
        search_nearest = True
        sort_neighbors = True
        lst = []

        if not query_point:
            return []

        search = K_neighbor_search(
            self.kd_tree, query_point, k, eps, search_nearest, self.metric(), sort_neighbors)
        search.k_neighbors(lst)
        print("For the point:\n\t", query_point, "\nThe nearest neighbors are:\n\t", lst)
        return [lst]

    # Check if edge can be added between two milestones in roadmap
    def _is_edge_legal(self, source: Point_3, destination: Point_3, clockwise: bool, partition_count=FT(Gmpq(100.0))):

        source_copy = np.array(point_3_to_xyz(source))
        destination_copy = np.array(destination[0])
        s_x, s_y, s_theta = point_3_to_xyz(source)
        d_x, d_y, d_theta = destination[0].x(), destination[0].y(), destination[0].y()
        x_diff, y_diff = s_x - d_x, s_y - d_y

        if (clockwise and d_theta >= s_theta) or (not clockwise and d_theta < s_theta):
            theta_diff = d_theta - s_theta
        elif clockwise and d_theta < s_theta:
            theta_diff = FT(-1) * (s_theta + FT((Gmpq(2.0))) * PI - d_theta)
        else:
            theta_diff = d_theta + (FT(Gmpq(2.0)) * PI - s_theta)
        diff_vector = np.array([x_diff, y_diff, theta_diff])

        # Sample nearby points on edge from source to destination, and check if they are legal
        t, h = FT(Gmpq(0.0)), FT(Gmpq(0.0))
        curr_point = source_copy
        # curr_point starts as source and converges to destination
        while not np.array_equal(curr_point, destination_copy) and h <= FT(Gmpq(1.0)):
            curr_point = source_copy + h * diff_vector
            t += FT(Gmpq(1.0))
            # partition count determines how we partition the edge from source to destination
            h = t / partition_count
            if not self._is_point_free(Point_3(curr_point[0], curr_point[1], curr_point[2])):
                return False
        return True

    # Grow roadmap by adding some milestones to it
    def grow_roadmap(self):

        milestones_found = 0

        while milestones_found < 50:  # self.milestones_count: FIXME
            # Generate a random position
            is_free_sample = False
            while not is_free_sample:
                curr_sample = generate_random_point(self.bounds)
                is_free_sample = self._is_point_free(curr_sample)
            milestones_found += 1
            self.add_milestone_to_roadmap(curr_sample)

        # Connect milestones found to its nearest neighbors, build edges of roadmap
        for milestone in list(self.roadmap.vertices):
            if isinstance(milestone, tuple):  # handle edge case
                milestone = milestone[0]
            print("Attempting to connect new milestone to roadmap: \n\t", milestone)
            self.connect_roadmap_vertex(milestone)

    # Connect a roadmap vertex to its k nearest neighbors
    def connect_roadmap_vertex(self, point, threshold=FT(Gmpq(50.0))):

        # TODO: Maybe consider connected components efficiency?
        nearest_neighbors = self._find_k_nearest_neighbors(point, NEIGHBORS)
        # Locality test
        nearest_node, dist = nearest_neighbors[0][0]

        if dist > threshold:
            pass
        sample_is_milestone = False
        for neighbor in nearest_neighbors[0]:
            # Attempt to connect CW and CCW edges from dest to neighbors
            if self.add_roadmap_edge(point, neighbor, True) or \
                    self.add_roadmap_edge(point, neighbor, False):
                sample_is_milestone |= True

    def add_roadmap_edge(self, point, neighbor, orientation_type) -> bool:

        if self._is_edge_legal(point, neighbor, orientation_type):
            weight = 1  # self.metric(neighbor, point, orientation_type) FIXME
            self.roadmap.add_edge(point, neighbor, weight, orientation_type)
            return True
        return False

    # TODO figure out threshold?
    def add_milestone_to_roadmap(self, point):

        self.roadmap.add_vertex(point)
        self.kd_tree.insert(point)


## --------------- Roadmap Graph  ------------------

class RoadmapGraph(object):
    def __init__(self):
        self.graph = defaultdict(list)

    def add_edge(self, source, dest, weight, orientation_type):
        self.graph[source].append(((source, dest), weight, orientation_type))
        self.graph[dest].append(((dest, source), weight, orientation_type))

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

    # TODO maybe use A*
    # Djikstra algorithm used to find shortest path on roadmap
    def djikstra(self, source) -> dict:
        distances = {n: float('infinity') for n in self.vertices}
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

    # Get shortest path from Djikstra
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


## --------------- Utilility Functions  ------------------

def generate_random_point(bounds):
    minX, maxX, minY, maxY = bounds[0], bounds[1], bounds[2], bounds[3]
    rand_x_coord = random.uniform(FT.to_double(minX), FT.to_double(maxX))
    rand_y_coord = random.uniform(FT.to_double(minY), FT.to_double(maxY))
    rand_theta_coord = Fraction(int(round(16 * random.uniform(0, 44 / 7))), 16)
    return Point_3(rand_x_coord, rand_y_coord, float(rand_theta_coord))


def point_3_to_xyz(p):
    return p.x(), p.y(), p.z()


def xyz_to_point_3(xyz):
    return Point_3(FT(Gmpq(xyz[0])), FT(Gmpq(xyz[1])), FT(Gmpq(xyz[2])))


## --------------- Run Algorithm  ------------------

def generate_path(path, length, obstacles, origin, destination):
    path = run_algorithm(length, obstacles, 100, 1, origin, destination)
    return path


# Grow PRM, look for a valid path: if it is not found, continue growing PRM and try again
def run_algorithm(rod_length, obstacles: List[Polygon_2], milestones_count, epsilon, source, dest):
    solution_found = False

    ### INITIALIZE PRM ###
    source = xyz_to_point_3(source)
    dest = xyz_to_point_3(dest)
    prm = PRM(rod_length, obstacles, milestones_count, FT(Gmpq(epsilon)), Euclidean_distance)
    prm.compute_bounds()
    prm.add_milestone_to_roadmap(source)
    prm.add_milestone_to_roadmap(dest)

    idx = 0
    while idx < len(obstacles):
        obstacles[idx] = tuples_list_to_polygon_2(obstacles[idx])
        idx += 1

    ### GROW PRM ###
    while not solution_found:
        prm.grow_roadmap()
        path = prm.roadmap.shortest_path(source, dest)

        if path:
            solution_found = True
    return path
