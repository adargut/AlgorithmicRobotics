import random
from fractions import Fraction
from typing import List

from arr2_epec_seg_ex import *
from collision_detection import *


# Return a uniformly sampled point in R2 x S1
def generate_random_point():
    # TODO find real bounds
    minX, maxX, minY, maxY = -5000, 5000, -5000, 5000
    rand_x_coord = random.uniform(minX, maxX)
    rand_y_coord = random.uniform(minY, maxY)
    rand_theta_coord = Fraction(int(round(16 * random.uniform(0, 22 / 7))), 16)
    return Point_3(rand_x_coord, rand_y_coord, float(rand_theta_coord))


# Check if uniformly sampled point is legal
def is_point_free(point: Point_3, l, polygon_list: List[Polygon_2], epsilon):
    return is_position_valid(point.x(), point.y(), point.z(), l, polygon_list, epsilon)


# Find nearest neighbors of sampled point, using KD-trees
def find_k_nearest_neighbors(query_point: Point_3, k, point_tree: Kd_tree, metric_function):
    eps = 1  # TODO maybe change epsilon?
    search_nearest = True
    sort_neighbors = True

    search = K_neighbor_search(point_tree, query_point, k, eps, search_nearest, metric_function(), sort_neighbors)

    lst = []
    return search.k_neighbors(lst)


# Check if edge can be added between two points in roadmap
def is_edge_legal(source: Point_3, destination: Point_3, sampling_distance):
    edge = Line_2(source, destination)


test = generate_random_point()
print(test)
