from arr2_epec_seg_ex import *
import networkx as nx
import math
import numpy as np

class Planner(object):
    def __init__(self, robots, obstacles):
        self.robots = list(map(Polygon_2, robots))
        self.obstacles = list(map(Polygon_2, obstacles))
        self._kd_tree = Kd_tree()
        self._graph = nx.DiGraph()
        self._bounds = self._compute_bounds()

    def _compute_bounds(self):
        minX, maxX, minY, maxY = math.inf, -math.inf, math.inf, -math.inf

        for obstacle in self.obstacles:
            for vertex in obstacle.vertices():
                x, y = vertex.x(), vertex.y()
                minX = min(minX, x)
                minY = min(minY, y)
                maxX = max(maxX, x)
                maxY = max(maxY, y)
        return [minX, maxX, minY, maxY]

    def _find_k_nearest_neighbors(self, query_point, k):
        # Point_d demonstration
        p_d = Point_d(4, [FT(1), FT(2), FT(3), FT(4)])
        print(p_d.dimension())
        print(p_d.cartesian(0))
        print(p_d[2])

        tree = Kd_tree([p_d])
        lst = []
        tree.points(lst)
        for i in range(10):
            tree.insert(Point_d(4, [FT(i), FT(i), FT(i), FT(i)]))

        query = Point_d(4, [FT(1), FT(2), FT(3), FT(4)])
        k = 5
        eps = FT(Gmpq(0.0))  # 0.0 for exact NN, otherwise approximate NN
        search_nearest = True  # set this value to False in order to search farthest
        sort_neighbors = False  # set this value to True in order to obtain the neighbors sorted by distance

        search = K_neighbor_search(tree, query, k, eps, search_nearest, \
                                   Euclidean_distance(), sort_neighbors)

        lst = []
        search.k_neighbors(lst)

    def _is_collision_free(self, source, target):
        raise NotImplemented

    def generate_path(self, destinations):
        return [[Point_2(1, 1), Point_2(2, 2)], [Point_2(3, 5), Point_2(5, 1)]]

    def RRT(self, source, target, n, eta):
        self._graph.clear()
        self._graph.add_node(source)
        for j in range(n):
            rand = self._get_free_sample()
            near = self._find_k_nearest_neighbors(rand, 1)
            new = self._steer(near, rand, eta)
            if self._is_collision_free(near, new):
                self._graph.add_node(new)
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
        raise NotImplemented


    def generate_random_point(self):
        minX, maxX, minY, maxY = self._bounds
        rand_x_coord = np.random.uniform(FT.to_double(minX), FT.to_double(maxX))
        rand_y_coord = np.random.uniform(FT.to_double(minY), FT.to_double(maxY))
        return Point_2(rand_x_coord, rand_y_coord)


def run_algorithm(robots, obstacles, destinations):
    planner = Planner(robots, obstacles)
    path = planner.generate_path(destinations)
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
4. COST = cost to get to point, distance in 4d?
    
"""