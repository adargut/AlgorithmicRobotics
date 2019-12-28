import random
import sys
import numpy as np
from fractions import Fraction
from typing import List
from conversions import *
from collision_detection import *
from math import sqrt
from collections import defaultdict, namedtuple
import heapq
import tqdm
import networkx as nx
import matplotlib.pyplot as plt
from arr2_epec_seg_ex import *

PI = FT(22 / 7)
NEIGHBORS = 11
PLUS_INF = FT(sys.maxsize)
MINUS_INF = FT((-1) * (sys.maxsize - 1))
SamplePoint = namedtuple('SamplePoint', ['x', 'y', 'z'])


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
        # self.roadmap = RoadmapGraph()
        self.roadmap = nx.DiGraph()
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
        eps = FT(Gmpq(0.0))  # TODO maybe change epsilon?
        search_nearest = True
        sort_neighbors = True
        lst = []

        if not query_point:
            return []

        search = K_neighbor_search(
            self.kd_tree, query_point, k, eps, search_nearest, self.metric(), sort_neighbors)
        search.k_neighbors(lst)
        return lst

    # Check if edge can be added between two milestones in roadmap
    def _is_edge_legal(self, source: Point_3, destination: Point_3, clockwise: bool, partition_count=FT(Gmpq(100.0))):
        s_x, s_y, s_theta = source.x(), source.y(), source.z()
        d_x, d_y, d_theta = destination.x(), destination.y(), destination.z()
        x_diff, y_diff = d_x - s_x, d_y - s_y

        if (clockwise and d_theta >= s_theta) or (not clockwise and d_theta < s_theta):
            theta_diff = d_theta - s_theta
        elif clockwise and d_theta < s_theta:
            theta_diff = FT(-1) * (s_theta + FT((Gmpq(2.0))) * PI - d_theta)
        else:
            theta_diff = d_theta + (FT(Gmpq(2.0)) * PI - s_theta)
        diff_vector = np.array([x_diff, y_diff, theta_diff])
        source_vector = np.array([s_x, s_y, s_theta])
        destination_vector = np.array([d_x, d_y, d_theta])
        # Sample nearby points on edge from source to destination, and check if they are legal
        t, h = FT(Gmpq(0.0)), FT(Gmpq(0.0))
        curr_point = source_vector
        # curr_point starts as source and converges to destination
        while not np.array_equal(curr_point, destination_vector) and h <= FT(Gmpq(1.0)):
            curr_point = source_vector + h * diff_vector
            t += FT(Gmpq(1.0))
            # partition count determines how we partition the edge from source to destination
            h = t / partition_count
            if not self._is_point_free(Point_3(curr_point[0], curr_point[1], curr_point[2])):
                return False
        return True

    # Grow roadmap by adding some milestones to it
    def grow_roadmap(self):
        samples_batch = 50
        samples = [generate_random_point(self.bounds) for i in range(samples_batch)]
        free_samples = [s for s in samples if self._is_point_free(xyz_to_point_3(s))]
        self.kd_tree.insert(list(map(xyz_to_point_3, free_samples)))
        self.roadmap.add_nodes_from(free_samples)
        # for s in free_samples:
        #     self.add_milestone_to_roadmap(s)

        # Connect milestones found to its nearest neighbors, build edges of roadmap
        for milestone in tqdm.tqdm(list(self.roadmap.nodes)):
            self.connect_roadmap_vertex(milestone)

    # Connect a roadmap vertex to its k nearest neighbors
    def connect_roadmap_vertex(self, node: SamplePoint, threshold=FT(Gmpq(50.0 ** 2))):

        # TODO: Maybe consider connected components efficiency?
        point = xyz_to_point_3(node)
        nearest_neighbors = self._find_k_nearest_neighbors(point, NEIGHBORS)
        # Locality test
        for neighbor, dist in nearest_neighbors:
            # print(f'{neighbor} is in distance {dist} from {point}')
            if FT(Gmpq(0.0)) < dist:  # <= threshold:
                # Attempt to connect CW and CCW edges from dest to neighbors
                if not self.add_roadmap_edge(point, neighbor, dist, True):
                    self.add_roadmap_edge(point, neighbor, dist, False)

    def add_roadmap_edge(self, point, neighbor, dist, orientation_type) -> bool:
        source = point_3_to_xyz(point)
        dest = point_3_to_xyz(neighbor)
        if not ((dest in self.roadmap[source]) or
                (source in self.roadmap[dest])):
            if self._is_edge_legal(point, neighbor, orientation_type):
                weight = sqrt(dist.to_double())  # self.metric(neighbor, point, orientation_type) FIXME
                self.roadmap.add_edge(source, dest, weight=weight, is_cw=orientation_type)
                self.roadmap.add_edge(dest, source, weight=weight, is_cw=not orientation_type)
                return True
        return False

    # TODO figure out threshold?
    def add_milestone_to_roadmap(self, point):
        self.roadmap.add_node(point)
        # self.kd_tree.insert(point)


## --------------- Roadmap Graph  ------------------

class RoadmapGraph(object):
    def __init__(self):
        self.graph = defaultdict(set)

    def add_edge(self, source, dest, weight, orientation_type):
        if self.is_exists(source) and self.is_exists(dest):
            self.graph[source].add(
                ((source, dest), (weight, orientation_type))
            )

    def add_vertex(self, v):
        self.graph[v] = set([])

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
        distances[source] = 0.0
        pq = [(0.0, source)]
        path = {}

        while len(pq) > 0:
            current_weight, min_node = heapq.heappop(pq)

            if current_weight > distances[min_node]:
                continue

            neighbors = self.neighbors(min_node)
            for edge, edge_data in neighbors:
                neighbor = edge[1]
                edge_weight, edge_orientation = edge_data
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
    minX, maxX, minY, maxY = -1000.0, 1000.0, -1000.0, 1000.0
    rand_x_coord = random.uniform(minX, maxX)
    rand_y_coord = random.uniform(minY, maxY)
    rand_theta_coord = Fraction(int(round(16 * random.uniform(0, 44 / 7))), 16)
    return SamplePoint(rand_x_coord, rand_y_coord, float(rand_theta_coord))
    # return Point_3(FT(rand_x_coord), FT(rand_y_coord), FT(float(rand_theta_coord)))


def point_3_to_xyz(p, to_double=True):
    if to_double:
        return p.x().to_double(), p.y().to_double(), p.z().to_double()
    return p.x(), p.y(), p.z()


def xyz_to_point_3(xyz):
    return Point_3(FT(Gmpq(xyz[0])), FT(Gmpq(xyz[1])), FT(Gmpq(xyz[2])))


## --------------- Run Algorithm  ------------------

def generate_path(path, length, obstacles, origin, destination):
    prm, path_points = run_algorithm(length, obstacles, 100, 1, origin, destination)
    if len(path_points) > 0:
        convert_sample_to_cgal = lambda sample: (FT(Gmpq(sample[0])), FT(Gmpq(sample[1])), FT(Gmpq(sample[2])))
        prev = path_points[0]
        path.append((*convert_sample_to_cgal(prev), True))
        for node in path_points[1:]:
            is_cw = prm.roadmap[prev][node]['is_cw']
            path.append((*convert_sample_to_cgal(node), is_cw))
            prev = node
    print(path)
    return path


# Grow PRM, look for a valid path: if it is not found, continue growing PRM and try again
def run_algorithm(rod_length, obstacles: List[Polygon_2], milestones_count, epsilon, source, dest):
    solution_found = False

    ### INITIALIZE PRM ###
    source = xyz_to_point_3(source)
    dest = xyz_to_point_3(dest)
    prm = PRM(rod_length, obstacles, milestones_count, FT(Gmpq(epsilon)), Euclidean_distance)
    prm.compute_bounds()

    idx = 0
    while idx < len(obstacles):
        obstacles[idx] = tuples_list_to_polygon_2(obstacles[idx])
        idx += 1

    ### GROW PRM ###
    prm.grow_roadmap()
    s = point_3_to_xyz(source)
    d = point_3_to_xyz(dest)
    prm.roadmap.add_node(s)
    prm.connect_roadmap_vertex(s)
    prm.roadmap.add_node(d)
    prm.connect_roadmap_vertex(d)
    print(" ### GROWN ROADMAP ###")
    if False:  # DEBUG
        g = prm.roadmap
        pos = nx.spring_layout(g)
        nx.draw_networkx_nodes([s], pos, node_color='b')
        nx.draw_networkx_nodes([d], pos, node_color='y')
        rest_of_the_nodes = set(g.nodes) - {s, d}
        nx.draw_networkx_nodes(rest_of_the_nodes, pos)
        nx.draw_networkx_edges(g, pos)
        plt.axis('off')
        plt.show()
    print("### FINDING PATH ###")
    print("from", source, "to", dest)
    # path = prm.roadmap.shortest_path(source, dest)
    path = nx.shortest_path(prm.roadmap, source=s, target=d, weight='weight')

    print(" ### GENERATED PATH ###\n", path)
    return prm, path
