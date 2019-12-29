import sys
import numpy as np
import tqdm
import networkx as nx
from arr2_epec_seg_ex import *
from collision_detection import *
from conversions import *
from typing import List
from math import sqrt
from collections import namedtuple
from time import time
from networkx import NetworkXNoPath

PI = FT(22 / 7)
NEIGHBORS = 15
PLUS_INF = FT(sys.maxsize)
MINUS_INF = FT((-1) * (sys.maxsize - 1))
SAMPLE_BATCH = 50
SamplePoint = namedtuple('SamplePoint', ['x', 'y', 'z'])


## --------------- PRM  ------------------

class PRM(object):

    def __init__(self, rod_length, obstacles: List[Polygon_2], milestones_count, epsilon, metric):

        self.rod_length = rod_length
        self.obstacles = obstacles
        self.milestones_count = milestones_count
        self.epsilon = epsilon
        self.metric = metric
        self.kd_tree = Kd_tree()
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

        eps = FT(Gmpq(0.0))
        search_nearest = True
        sort_neighbors = True
        lst = []

        if not query_point:
            return []

        # Handle both custom metric and Euclidean Distance
        if isinstance(self.metric, Euclidean_distance):
            search = K_neighbor_search(
                self.kd_tree, query_point, k, eps, search_nearest, self.metric, sort_neighbors)
        else:
            search = K_neighbor_search_python(
                self.kd_tree, query_point, k, eps, search_nearest, self.metric, sort_neighbors)
        search.k_neighbors(lst)
        return lst

    # Check if edge can be added between two milestones in roadmap
    def _is_edge_legal(self, source: Point_3, destination: Point_3, clockwise: bool, partition_count=FT(Gmpq(50.0))):

        s_x, s_y, s_theta = source.x(), source.y(), source.z()
        d_x, d_y, d_theta = destination.x(), destination.y(), destination.z()
        x_diff, y_diff = d_x - s_x, d_y - s_y
        ccw = not clockwise

        if (ccw and d_theta >= s_theta) or (clockwise and d_theta < s_theta):
            theta_diff = d_theta - s_theta
        elif ccw and d_theta < s_theta:
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

        samples = [generate_random_point(self.bounds) for i in range(self.milestones_count)]
        free_samples = [s for s in samples if self._is_point_free(xyz_to_point_3(s))]

        self.kd_tree.insert(list(map(xyz_to_point_3, free_samples)))
        self.roadmap.add_nodes_from(free_samples)

        # Connect milestones found to its nearest neighbors, build edges of roadmap
        for milestone in tqdm.tqdm(list(self.roadmap.nodes)):
            self.connect_roadmap_vertex(milestone)

    def increment_milestone_count(self):

        self.milestones_count *= 2

    # Connect a roadmap vertex to its k nearest neighbors
    def connect_roadmap_vertex(self, node: SamplePoint, threshold=FT(Gmpq(100000.0 ** 2))):

        point = xyz_to_point_3(node)
        nearest_neighbors = self._find_k_nearest_neighbors(point, NEIGHBORS)
        # Locality test
        for neighbor, dist in nearest_neighbors:
            if FT((Gmpq(0.0))) < dist <= threshold:
                # Attempt to connect CW and CCW edges from dest to neighbors
                if not self.add_roadmap_edge(point, neighbor, dist, True):
                    self.add_roadmap_edge(point, neighbor, dist, False)

    def add_roadmap_edge(self, point, neighbor, dist, orientation_type) -> bool:

        source = point_3_to_xyz(point)
        dest = point_3_to_xyz(neighbor)
        if not ((dest in self.roadmap[source]) or
                (source in self.roadmap[dest])):
            if self._is_edge_legal(point, neighbor, orientation_type):
                weight = sqrt(dist.to_double())
                self.roadmap.add_edge(source, dest, weight=weight, is_cw=orientation_type)
                self.roadmap.add_edge(dest, source, weight=weight, is_cw=not orientation_type)
                return True
        return False

    def add_milestone_to_roadmap(self, point):

        self.roadmap.add_node(point)


## --------------- Utilility Functions  ------------------

def generate_random_point(bounds):
    minX, maxX, minY, maxY = bounds
    rand_x_coord = np.random.uniform(FT.to_double(minX), FT.to_double(maxX))
    rand_y_coord = np.random.uniform(FT.to_double(minY), FT.to_double(maxY))
    rand_theta_coord = np.random.uniform(0, FT.to_double(PI))
    return SamplePoint(rand_x_coord, rand_y_coord, float(rand_theta_coord))


def point_3_to_xyz(p, to_double=True):
    if to_double:
        return p.x().to_double(), p.y().to_double(), p.z().to_double()
    return p.x(), p.y(), p.z()


def xyz_to_point_3(xyz):
    return Point_3(FT(Gmpq(xyz[0])), FT(Gmpq(xyz[1])), FT(Gmpq(xyz[2])))


## --------------- Manhattan Distance  ------------------

# The following function returns the transformed distance between two points
# (for Euclidean distance the transformed distance is the squared distance)
def transformed_distance(p1, p2):
    assert isinstance(p1, Point_3)
    assert isinstance(p2, Point_3)

    return abs(p1.x() - p2.x()) + abs(p1.y() - p2.y()) + abs(p1.z() - p2.z())

# The following function returns the transformed distance between the query
# point q and the point on the boundary of the rectangle r closest to q.
def min_distance_to_rectangle(q, r):
    assert isinstance(r, Kd_tree_rectangle)
    assert isinstance(q, Point_3)
    dist = FT(Gmpq(0.0))

    if r.min_coord(0) > q.x():
        dist += r.min_coord(0) - q.x()
    if r.max_coord(0) < q.x():
        dist += q.x() - r.max_coord(0)
    if r.min_coord(1) > q.x():
        dist += r.min_coord(1) - q.x()
    if r.max_coord(1) < q.x():
        dist += q.x() - r.max_coord(1)
    if r.min_coord(2) > q.x():
        dist += r.min_coord(2) - q.x()
    if r.max_coord(2) < q.x():
        dist += q.x() - r.max_coord(2)


# The following function returns the transformed distance between the query
# point q and the point on the boundary of the rectangle r furthest to q.
def max_distance_to_rectangle(q, r):
    assert isinstance(r, Kd_tree_rectangle)
    assert isinstance(q, Point_3)
    dist = FT(Gmpq(0.0))

    if (r.max_coord(0) - q.x()) > (q.x() - r.min_coord(0)):
        dist += (r.max_coord(0) - q.x())
    else:
        dist += q.x() - r.min_coord(1)
    if (r.max_coord(1) - q.x()) > (q.x() - r.min_coord(1)):
        dist += (r.max_coord(1) - q.x())
    else:
        dist += q.x() - r.min_coord(1)
    if (r.max_coord(2) - q.x()) > (q.x() - r.min_coord(2)):
        dist += (r.max_coord(2) - q.x())
    else:
        dist += q.x() - r.min_coord(2)


# The following function returns the transformed distance for a value d
# Fo example, if d is a value computed using the Euclidean distance, the transformed distance should be d*d
def transformed_distance_for_value(d):
    return d


# The following function returns the inverse of the transformed distance for a value d
# Fo example, if d is a sqaured distance value then its inverse should be sqrt(d)
def inverse_of_transformed_distance_for_value(d):
    return d


manhattan_distance = Distance_python(transformed_distance,
                                     min_distance_to_rectangle,
                                     max_distance_to_rectangle,
                                     transformed_distance_for_value,
                                     inverse_of_transformed_distance_for_value)

## --------------- Run Algorithm  ------------------

def generate_path(path, length, obstacles, origin, destination):
    save_dest = destination
    prm, path_points = run_algorithm(length, obstacles, 100, 1, origin, destination)
    if len(path_points) > 0:
        convert_sample_to_cgal = lambda sample: (FT(Gmpq(sample[0])), FT(Gmpq(sample[1])), FT(Gmpq(sample[2])))
        prev = path_points[0]
        path.append((*convert_sample_to_cgal(prev), True))
        for node in path_points[1:]:
            is_cw = prm.roadmap[prev][node]['is_cw']
            path.append((*convert_sample_to_cgal(node), is_cw))
            prev = node
    path[-1] = path[-1][0], path[-1][1], FT(Gmpq(save_dest[2])), path[-1][3]
    return path


# Grow PRM, look for a valid path: if it is not found, continue growing PRM and try again
def run_algorithm(rod_length, obstacles: List[Polygon_2], milestones_count, epsilon, source, dest):
    solution_found = False

    ### INITIALIZE PRM ###
    source = xyz_to_point_3(source)
    dest = xyz_to_point_3(dest)
    s = point_3_to_xyz(source)
    d = point_3_to_xyz(dest)

    prm = PRM(rod_length, obstacles, milestones_count, FT(Gmpq(epsilon)), Euclidean_distance())
    prm.compute_bounds()

    prm.roadmap.add_node(s)
    prm.connect_roadmap_vertex(s)
    prm.roadmap.add_node(d)
    prm.connect_roadmap_vertex(d)

    start_time = time()
    epoch = 0
    ### GROW PRM ###
    while not solution_found:
        idx = 0
        while idx < len(obstacles):
            obstacles[idx] = tuples_list_to_polygon_2(obstacles[idx])
            idx += 1

        epoch += 1
        print("### EPOCH NUMBER", epoch, "###")
        prm.grow_roadmap()
        print("\t### GROWING ROADMAP ###")
        print("\t\tSuccessfully added", prm.milestones_count, "new samples. "
                                                              "Kd-tree size is now:",
              len([p for p in prm.kd_tree.points()]),
              "num of edges:", len(prm.roadmap.edges))
        prm.increment_milestone_count()
        print("\t### FINDING PATH ###")
        print("\t\tfrom", source, "to", dest)
        try:
            path = nx.shortest_path(prm.roadmap, source=s, target=d, weight='weight')
        except NetworkXNoPath:
            path = []
        print("\t### ATTEMPTING TO GENERATE PATH... ##")
        if path:
            solution_found = True

        idx = 0
        while idx < len(obstacles):
            obstacles[idx] = polygon_2_to_tuples_list(obstacles[idx])
            idx += 1

    end_time = time()
    print("### SUCCESS: GENERATED PATH IN", end_time - start_time, "SECONDS ###")
    print(path)
    return prm, path
