from typing import Tuple

from arr2_epec_seg_ex import *
import networkx as nx
import math
import numpy as np
from ms_polygon_segment import *
import tqdm
from ms_polygon_segment import minkowski_sum_polygon_point
import matplotlib.pyplot as plt
from math import sqrt

N = 300


class Nearest(object):
    SIZE = 50

    def __init__(self):
        self.buffer = [None] * self.SIZE
        self.buffer_size = 0
        self.tree = Kd_tree()

    def get_nearest(self, query_point):
        min_point = None
        min_squared_dist = FT(math.inf)
        if self.buffer_size > 0:
            distance = Euclidean_distance()
            distances = map(lambda p: (p, distance.transformed_distance(p, query_point)),
                            self.buffer[:self.buffer_size])
            min_point, min_squared_dist = min(distances, key=lambda t: t[1])
        if self.tree.size() > 0:
            nearest_point, nearest_squared_dist = self._find_k_nearest_neighbors(query_point, k=15)
            if nearest_squared_dist < min_squared_dist:
                return nearest_point
        return min_point

    def add_node(self, node):
        if self.buffer_size < self.SIZE:
            self.buffer[self.buffer_size] = node
            self.buffer_size += 1
        else:
            self.tree.insert(self.buffer)
            self.buffer_size = 0


class Planner(object):
    def __init__(self, robots, obstacles):
        self.robots = list(map(Polygon_2, robots))
        self.obstacles = list(map(Polygon_2, obstacles))
        self._polygon_set = Polygon_set_2()
        self._polygon_set.join_polygons(self.obstacles)
        self._kd_tree = Kd_tree()
        self._graph = nx.DiGraph()
        self._bounds = self._compute_bounds()
        self.solution_found = False
        self._sample_size = 100
        self.src = None
        self.dst = None

    def _compute_bounds(self):
        minX, maxX, minY, maxY = FT(math.inf), FT(-math.inf), FT(math.inf), FT(-math.inf)

        for obstacle in self.obstacles:
            for vertex in obstacle.vertices():
                x, y = vertex.x(), vertex.y()
                minX = min(minX, x)
                minY = min(minY, y)
                maxX = max(maxX, x)
                maxY = max(maxY, y)
        return [minX, maxX, minY, maxY]

    def _find_k_nearest_neighbors(self, query_point, k=15):
        eps = FT(Gmpq(1.0))  # 0.0 for exact NN, otherwise approximate NN
        search_nearest = True  # set this value to False in order to search farthest
        sort_neighbors = True  # set this value to True in order to obtain the neighbors sorted by distance

        search = K_neighbor_search(self._kd_tree, query_point, k, eps, search_nearest, Euclidean_distance(),
                                   sort_neighbors)

        lst = []
        search.k_neighbors(lst)
        return lst

    def _do_intersect(self, p1, p2, source, target):
        # fix p1 and move p2 relatively
        ps = Polygon_set_2()
        p1 = minkowski_sum_polygon_point(p1, source[0])
        ps.insert(p1)
        s = source[1]
        t = target[1] + Vector_2(target[0], source[0])
        if s == t:
            ms = minkowski_sum_polygon_point(p2, s)
        else:
            segment = Segment_2(s, t)
            ms = minkowski_sum_polygon_segment(p2, segment)
        return ps.do_intersect(ms)

    def _is_collision_free(self, source, target):
        s1, s2 = from_point_d(source)
        t1, t2 = from_point_d(target)

        # move robot 1
        r1_start = minkowski_sum_polygon_point(self.robots[0], s1)
        r1_dest = minkowski_sum_polygon_point(self.robots[0], t1)
        # Find convex hull of moved r1
        r1_movement = []
        r1_as_points = []
        for p in self.polygon_2_to_tuples_list(r1_start.outer_boundary()):
            r1_as_points.append(Point_2(p[0], p[1]))
        for p in self.polygon_2_to_tuples_list(r1_dest.outer_boundary()):
            r1_as_points.append(Point_2(p[0], p[1]))
        # ch stores result for convex hull
        convex_hull_2(r1_as_points, r1_movement)

        # move robot 2
        r2_start = minkowski_sum_polygon_point(self.robots[1], s2)
        r2_dest = minkowski_sum_polygon_point(self.robots[1], t2)
        # Find convex hull of moved r2
        r2_movement = []
        r2_as_points = []
        for p in self.polygon_2_to_tuples_list(r2_start.outer_boundary()):
            r2_as_points.append(Point_2(p[0], p[1]))
        for p in self.polygon_2_to_tuples_list(r2_dest.outer_boundary()):
            r2_as_points.append(Point_2(p[0], p[1]))
        # r2_ch stores result for convex hull
        convex_hull_2(r2_as_points, r2_movement)

        # Conversion to match C++ signature
        r1_movement = Polygon_2(r1_movement)
        r2_movement = Polygon_2(r2_movement)

        return True
        # Check for intersection between the robots
        # return not self._do_intersect(self.robots[0], self.robots[1], [s1, s2], [t1, t2])

    def _is_valid_point(self, point):
        p1, p2 = from_point_d(point)

        # Move r1 to point p1 and r2 to point p2
        r1 = minkowski_sum_polygon_point(self.robots[0], p1)
        r2 = minkowski_sum_polygon_point(self.robots[1], p2)

        # Check if robot intersect obstacles
        if self._polygon_set.do_intersect(r1) or self._polygon_set.do_intersect(r2):
            return False

        return True

        # # Check if robots intersect each other
        # ps = Polygon_set_2(r1)
        # return not ps.do_intersect(r2)

    def polygon_2_to_tuples_list(self, polygon):
        lst = [(p.x().to_double(), p.y().to_double()) for p in polygon.vertices()]
        return lst

    def _grow_kd_tree(self, samples):
        self._kd_tree.insert(samples)

    def _add_node_to_tree(self, node):
        self._graph.add_node(node)

    def generate_path(self, n):
        # midpoint_1 = center_point(self.robots[0])
        # midpoint_2 = center_point(self.robots[1])
        # source = to_point_d(midpoint_1, midpoint_2)
        #
        # target = to_point_d(midpoint_2, midpoint_2)

        # self._graph.add_node(Point_d(4, [midpoint_1.x(), midpoint_1.y(), midpoint_2.x(), midpoint_2.y()]))
        # self._graph.add_node(Point_d(4, [midpoint_2.x(), midpoint_2.y(), midpoint_2.x(), midpoint_2.y()]))
        # self._graph.add_node(midpoint_2)
        self._graph.add_node(self.src)
        self._graph.add_node(self.dst)

        # self.RRT(source, target, n)
        self.grow_roadmap()

        if nx.has_path(self._graph, self.src, self.dst):
            self.dst = Point_d(4, [FT(12.0), FT(12.0), FT(12.0), FT(12.0)])
            if not nx.has_path(self._graph, self.src, self.dst):
                self.solution_found = False
                return None
            path = nx.shortest_path(self._graph, self.src, self.dst)
            if len(path) > 0:
                path = [from_point_d(p) for p in path]
            self.solution_found = True
            return path

        # Check if edge can be added between two milestones in roadmap
        def _is_edge_legal(self, source: Point_3, destination: Point_3, clockwise: bool, dist):

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

            source_vector = np.array([s_x, s_y, s_theta])
            destination_vector = np.array([d_x, d_y, d_theta])
            diff_vector = np.array([x_diff, y_diff, theta_diff])
            dist = x_diff * x_diff + y_diff * y_diff + theta_diff * theta_diff * self.rod_length
            dist = sqrt(FT.to_double(dist))
            partition_count = FT(dist) / self.epsilon
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

    def _is_collision_free(self, source, target):
        s1, s2 = from_point_d(source)
        t1, t2 = from_point_d(target)

        # move robot 1
        r1_start = minkowski_sum_polygon_point(self.robots[0], s1)
        r1_dest = minkowski_sum_polygon_point(self.robots[0], t1)
        # Find convex hull of moved r1
        r1_movement = []
        r1_as_points = []
        for p in self.polygon_2_to_tuples_list(r1_start.outer_boundary()):
            r1_as_points.append(Point_2(p[0], p[1]))
        for p in self.polygon_2_to_tuples_list(r1_dest.outer_boundary()):
            r1_as_points.append(Point_2(p[0], p[1]))
        # ch stores result for convex hull
        convex_hull_2(r1_as_points, r1_movement)

        # move robot 2
        r2_start = minkowski_sum_polygon_point(self.robots[1], s2)
        r2_dest = minkowski_sum_polygon_point(self.robots[1], t2)
        # Find convex hull of moved r2
        r2_movement = []
        r2_as_points = []
        for p in self.polygon_2_to_tuples_list(r2_start.outer_boundary()):
            r2_as_points.append(Point_2(p[0], p[1]))
        for p in self.polygon_2_to_tuples_list(r2_dest.outer_boundary()):
            r2_as_points.append(Point_2(p[0], p[1]))
        # r2_ch stores result for convex hull
        convex_hull_2(r2_as_points, r2_movement)

        # Conversion to match C++ signature
        r1_movement = Polygon_2(r1_movement)
        r2_movement = Polygon_2(r2_movement)

        # Check for intersection between robots and obstacles
        if self._polygon_set.do_intersect(r1_movement) or self._polygon_set.do_intersect(r2_movement):
            return False
        return True

    # connect a vertex to its neighbors
    def connect_roadmap_vertex(self, vertex):
        neighbors = self._find_k_nearest_neighbors(vertex)

        for neighbor, dist in neighbors:
            if self._is_collision_free(vertex, neighbor):
                self._graph.add_edge(vertex, neighbor, weight=(sqrt(dist.to_double())))
                self._graph.add_edge(neighbor, vertex, weight=(sqrt(dist.to_double())))

    def _move_target(self, x, y):
        self.src[2], self.src[3] = x, y

    def grow_roadmap(self):
        samples = [self._get_free_sample() for i in range(self._sample_size)]
        for sample in samples:
            self._graph.add_node(sample)
        self._kd_tree.insert(list(samples))

        for milestone in tqdm.tqdm(list(self._graph.nodes)):
            self.connect_roadmap_vertex(milestone)

        if False:  # debug
            g = self._graph
            pos = nx.spring_layout(g)
            nx.draw_networkx_nodes([self.src], pos, node_color='r')
            nx.draw_networkx_nodes([self.dst], pos, node_color='y')
            rest_of_the_nodes = set(g.nodes) - {self.src, self.dst}
            nx.draw_networkx_nodes(rest_of_the_nodes, pos)
            nx.draw_networkx_edges(g, pos)
            plt.axis('off')
            plt.show()

        self.connect_roadmap_vertex(self.dst)
        self.connect_roadmap_vertex(self.src)

    def _get_free_sample(self):
        sample = None
        valid = False

        while not valid:
            robot_2 = Point_2(self.src[2], self.src[3])
            robot_1 = self.generate_random_point()
            sample = to_point_d(robot_1, robot_2)
            if self._is_valid_point(sample):
                valid = True
                sample = to_point_d(robot_1, robot_2)
        return sample

    def generate_random_point(self):
        minX, maxX, minY, maxY = self._bounds
        rand_x_coord = np.random.uniform(FT.to_double(minX), FT.to_double(maxX))
        rand_y_coord = np.random.uniform(FT.to_double(minY), FT.to_double(maxY))
        return Point_2(rand_x_coord, rand_y_coord)


def run_algorithm(robots, obstacles):
    planner = Planner(robots, obstacles)
    robots = list(map(Polygon_2, robots))
    midpoint_1 = center_point(robots[0])
    midpoint_2 = center_point(robots[1])

    source = to_point_d(midpoint_1, midpoint_2)
    target = to_point_d(midpoint_2, midpoint_2)

    planner.src = source
    planner.dst = target

    while not planner.solution_found:
        path = planner.generate_path(N)
    return path


def generate_path(path, robots, obstacles, destination):
    print(path, robots, obstacles, destination)
    out_path = run_algorithm(robots, obstacles)
    print(destination, type(destination))
    for p in out_path:
        path.append(p)
        pass


def to_point_d(p1: Point_2, p2: Point_2) -> Point_d:
    return Point_d(4, [p1.x(), p1.y(), p2.x(), p2.y()])


def from_point_d(p: Point_d) -> Tuple[Point_2, Point_2]:
    return Point_2(p[0], p[1]), Point_2(p[2], p[3])


def center_point(polygon):
    count = FT(polygon.size())
    x = sum([p.x() for p in polygon.vertices()], FT(0))
    y = sum([p.y() for p in polygon.vertices()], FT(0))
    return Point_2(x / count, y / count)

##################
