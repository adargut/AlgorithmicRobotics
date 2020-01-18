from arr2_epec_seg_ex import *
import networkx as nx
import math
import random
import numpy as np
import linear_path_intersection_test
import tqdm

N = 100


class Planner(object):
    def __init__(self, robots, obstacles, destinations):
        self.robots = list(map(Polygon_2, robots))
        self.obstacles = Polygon_set_2()
        self.obstacles.insert_polygons(list(map(Polygon_2, obstacles)))
        
        self.destinations = destinations
        self._kd_tree = Kd_tree()
        self._neighbors_buffer = []
        self._graph = nx.DiGraph()
        self._bounds = self._compute_bounds()

        self._aabb = {}
        

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


    def _find_k_nearest_neighbors(self, query_point, k=5):
        eps = FT(Gmpq(0.0))  # 0.0 for exact NN, otherwise approximate NN
        search_nearest = True  # set this value to False in order to search farthest
        sort_neighbors = False  # set this value to True in order to obtain the neighbors sorted by distance

        search = K_neighbor_search(self._kd_tree, query, k, eps, search_nearest, \
                                   Euclidean_distance(), sort_neighbors)

        lst = []
        search.k_neighbors(lst)
        return lst

    def _is_collision_free(self, source, target):
        return True # FIXME: Actually check if the point is in a free space
        s1 = Point_2(source[0], source[1])
        s2 = Point_2(source[1], source[2])

        t1 = Point_2(target[0], target[1])
        t2 = Point_2(target[1], target[2])

        # move robot 1
        r1_start = minkowski_sum_polygon_point(self.robots[0], s1)
        r1_dest = minkowski_sum_polygon_point(self.robots[0], t1)
        r1_movement = convex_hull_2([r1_start, r1_dest])

        # move robot 2
        r2_start = minkowski_sum_polygon_point(self.robots[1], s2)
        r2_dest = minkowski_sum_polygon_point(self.robots[1], t2)
        r2_movement = convex_hull_2([r2_start, r2_dest])

        if self.obstacles.do_intersect(r1_movement) or self.obstacles.do_intersect(r2_movement):
            return False

        return not linear_path_intersection_test.do_intersect(self.robots[0], self.robots[1], [s1, s2], [t1, t2])
        
    def _is_valid_point(self, point):
        return True  # FIXME: Actually check if the point is in a free space
        p1 = Point_2(point[0], point[1])
        p2 = Point_2(point[1], point[2])

        r1 = minkowski_sum_polygon_point(self.robots[0], p1)
        r2 = minkowski_sum_polygon_point(self.robots[1], p2)

        
        if self.obstacles.do_intersect(r1) or self.obstacles.do_intersect(r2):
            return False
        
        ps = Polygon_set_2(p1)
        return not ps.do_intersect(p2)

    # def calculate_free_space():
    #     self._aabb_dict = {Bbox_2(obstacle.vertices()): obstacle for obstacle in self.obstacles()}

    def _add_node_to_tree(self, node):
        self._graph.add_node(node)
        self._kd_tree.insert(node)  # FIXME: inefficient!

    def generate_path(self, n):
        midpoint_1 = sum(self.robots[0].vertices()) / FT(self.robots[0].size())
        midpoint_2 = sum(self.robots[1].vertices()) / FT(self.robots[1].size())
        source = Point_d(4, [midpoint_1[0], midpoint_1[1], midpoint_2[0], midpoint_2[1]])

        self._kd_tree = Kd_tree([source])
        self._graph.add_node(source)

        target = Point_d(4, self.destinations[0].extend(self.destinations[1]))

        self.RRT(source, self.destinations, target, n)

        path = nx.shortest_path(self._graph, source, target)
        if len(path) > 0:
            path = [[Point_2(p[0], p[1]), Point_2(p[2], p[3])] for p in path]
        return path
        #return [[Point_2(1, 1), Point_2(2, 2)], [Point_2(3, 5), Point_2(5, 1)]]

    def RRT(self, source, target, n, eta=1):
        self._graph.clear()
        self._graph.add_node(source)
        for j in tqdm.tqdm(range(n)):
            rand = self._get_free_sample()
            near, _ = self._find_k_nearest_neighbors(rand, k=1)
            new = self._steer(near, rand, eta)
            if self._is_collision_free(near, new):
                self._add_node_to_tree(new)
                self._graph.add_edge(near, new)

    def RRT_star(self, source, target, n, eta):
        self._graph.clear()
        self._graph.add_node(source)

        def radius(n):
            D = 4
            GAMMA = 2.5
            return GAMMA * (math.log2(n) / n) ** (1.0 / (D + 1))

        for j in range(n):
            rand_point = self._get_free_sample()
            near = self._find_k_nearest_neighbors(rand_point, 1)
            new_point = self._steer(near, rand_point, eta)
            if self._is_collision_free(near, new_point):
                V = self._graph.number_of_nodes()
                # new_parent = self._find_best_new_parent(new_point, r(V))
                neighbors = NEAR(new_point, V, min(radius(V), eta))
                self._graph.add_node(new_point)
                min_point = near
                min_cost = COST(near) + squared_distance(new_point, near)
                for near in neighbors:
                    if self._is_collision_free(near, new_point):
                        curr_cost = COST(near) + squared_distance(new_point, near)
                        if curr_cost < min_cost:
                            min_point = near
                            min_cost = curr_cost
                self._graph.add_edge(min_point, new_point)
                # rewire the new parent
                for near in neighbors:
                    if self._is_collision_free(near, new_point):
                        if COST(new_point) + squared_distance(new_point, near) < COST(near):
                            parent = [self._graph.predecessors(near)][0]
                            self._graph.remove_edge(parent, near)
                            self._graph.add_edge(new_point, near)

    def _get_free_sample(self):
        sample = None
        valid = False
        GOAL_BIAS = 5
        if np.random.randint(0, 100) <= GOAL_BIAS:
            return Point_d(4, [self.destinations[0].x(), self.destinations[0].y(), self.destinations[1].x(), self.destinations[1].y()])
        while not valid:
            robot_1 = self.generate_random_point()
            robot_2 = self.generate_random_point()
            if self._is_valid_point(robot_1) and self._is_valid_point(robot_2):
                valid = True
                sample = Point_d(4, [robot_1.x(), robot_1.y(), robot_2.x(), robot_2.y()])
        return sample

    def _steer(self, source, target, eta):
        s1 = Point_2(source[0], source[1])
        s2 = Point_2(source[1], source[2])

        t1 = Point_2(target[0], target[1])
        t2 = Point_2(target[1], target[2])

        v1 = Vector_2(s1, t1)
        v2 = Vector_2(s2, t2)

        v_len = FT(math.sqrt(v1.squared_length().to_double() + v2.squared_length().to_double()))

        if v_len <= eta:
            return target
        else:
            p1 = s1 + v1 / FT(v_len * eta)
            p2 = s2 + v2 / FT(v_len * eta)
            return Point_d(4, [p1[0], p1[1], p2[0], p2[1]])


    def generate_random_point(self):
        minX, maxX, minY, maxY = self._bounds
        rand_x_coord = np.random.uniform(FT.to_double(minX), FT.to_double(maxX))
        rand_y_coord = np.random.uniform(FT.to_double(minY), FT.to_double(maxY))
        return Point_2(rand_x_coord, rand_y_coord)



def move_robot(prev_center, new_center, robot):
    v = Vector_2(prev_center, new_center)


def run_algorithm(robots, obstacles, destinations):
    planner = Planner(robots, obstacles, destinations)
    path = planner.generate_path()
    return path

def generate_path(path, robots, obstacles, destination):
    print(path, robots, obstacles, destination)
    out_path = run_algorithm(robots, obstacles, destination)
    for p in out_path:
        path.append(p)

##################

"""
TODO:
1. Use a 4 dimesnsional space for the space
    Kd tree, Distance, etc
2. Write a proper collision detection
    transform 4d point to 2 2d points
    for each check with obstacles
    UNDER ASSUMPTION OF SIMULTANEOUS MOVEMENT AND FIXED SPEED
    check if paths collide with each other
3. NEAR = get nearest neighbors in that radius!
4. Cost(v) = Cost(Parent(v)) + c(Line(Parent(v), v)). Cost(root) = 0



Collision Detection:
For every polygon, compute AABB
Each face will have a different "name"
Put into arrangement A
Overlay - Free space will have no name
        Intersection witn n AABBs will have n "names"

Use point location on the arrangement A to check where the point is with which Boxes to compute intersection!

"""