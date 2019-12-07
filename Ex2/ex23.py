import heapq
import math
from collections import defaultdict
from queue import Queue
from typing import List

from arr2_epec_seg_ex import *  # Arr_overlay_traits, Arrangement_2, Point_2
from conversions import *

FREE_FACE = 0
FORBIDDEN_FACE = 1


def generate_path(path, robot, obstacles, destination):
    assert obstacles is not None
    assert robot is not None
    assert destination is not None
    # get polygonal objects
    obs = [tuples_list_to_polygon_2(o) for o in obstacles]
    ref_point = xy_to_point_2(robot[0][0], robot[0][1])
    r = tuples_list_to_polygon_2(robot)
    d = xy_to_point_2(*destination)
    source = ref_point

    conf_space = compute_configuration_space(ref_point, obs, r, d)
    # compute trapezoid decomposition of conf_space
    trapezoid_space = trapezoid_decompose(conf_space)
    # build roadmap
    roadmap = build_roadmap(trapezoid_space)
    # bfs on roadmap
    roadmap_path = roadmap_bfs(source, d, roadmap, trapezoid_space)
    for i in range(1, len(roadmap_path) - 1):
        roadmap_path[i] = xy_to_point_2(roadmap_path[i][0], roadmap_path[i][1])
    # save path to file
    with open("path0.txt", 'w') as savefile:
        savefile.truncate(0)
        for point in roadmap_path:
            line = str(int(point_2_to_xy(point)[0])) + " " + str(int(point_2_to_xy(point)[1])) + "\n"
            path.append(point)
            savefile.write(line)


def reflect_polygon(polygon: Polygon_2) -> Polygon_2:
    out = []
    for v in polygon.vertices():
        x, y = point_2_to_xy(v)
        out.append((-x, -y))
    reflected = tuples_list_to_polygon_2(out)
    if reflected.orientation() == CLOCKWISE:
        reflected.reverse_orientation()
    return reflected


def obstacles_to_arrangement(obstacles):
    arr = Arrangement_2()
    lst = []
    for polygon in obstacles:
        for e in polygon.edges():
            lst.append(Curve_2(e))
    insert(arr, lst)
    return arr


def mark_free_faces(face: Face, status):
    face.set_data(status)
    if face.number_of_inner_ccbs() > 0:
        for inner_ccb in face.inner_ccbs():
            for he in inner_ccb:
                new_face = he.twin().face()
                mark_free_faces(new_face, not status)
                break  # we only care about one edge for each face


def compute_configuration_space(ref_point: Point_2, obstacles: List[Polygon_2],
                                robot: Polygon_2, destination: Point_2) -> Arrangement_2:
    # move robot top to (0, 0)
    ref_point = point_2_to_xy(ref_point)
    top_x, top_y = ref_point[0], ref_point[1]
    moved_robot_points = [(x - top_x, y - top_y) for (x, y) in polygon_2_to_tuples_list(robot)]
    start_robot = tuples_list_to_polygon_2(moved_robot_points)
    reflected_robot = reflect_polygon(start_robot)
    arr = Arrangement_2()

    for obstacle in obstacles:
        # create obstacle
        obs_arr = Arrangement_2()
        msum = minkowski_sum_2(reflected_robot, obstacle)
        boundary = msum.outer_boundary()
        insert_non_intersecting_curves(obs_arr, list(map(Curve_2, boundary.edges())))
        # set up face data for obstacle
        mark_free_faces(obs_arr.unbounded_face(), FREE_FACE)
        # overlay the new obstacle over the previous data
        arr = arr_overlay(arr, obs_arr)

    # Bounding Box
    bbox_arr = Arrangement_2()
    robot_bbox = robot.bbox()

    offset = 0  # robot_bbox.max().to_double()
    xmin = 0 - offset
    xmax = 2000 + offset
    ymin = 0 - offset
    ymax = 2000 + offset

    bounds = [
        Curve_2(xy_to_point_2(xmin, ymin), xy_to_point_2(xmin, ymax)),
        Curve_2(xy_to_point_2(xmin, ymin), xy_to_point_2(xmax, ymin)),
        Curve_2(xy_to_point_2(xmax, ymax), xy_to_point_2(xmin, ymax)),
        Curve_2(xy_to_point_2(xmax, ymax), xy_to_point_2(xmax, ymin)),
    ]

    # Bbox is a reverse obstacle - only allowed inside
    insert_non_intersecting_curves(bbox_arr, bounds)
    mark_free_faces(bbox_arr.unbounded_face(), FORBIDDEN_FACE)
    arr = arr_overlay(arr, bbox_arr)
    return arr


def arr_overlay(arr1, arr2):
    def layer(x, y):
        x = 0 if x is None else x
        y = 0 if y is None else y
        return int(bool(x) or bool(y))

    def empty(x, y):
        return None

    traits = Arr_overlay_traits(empty, empty, empty, empty, empty, empty, empty, empty, empty, layer)
    res = Arrangement_2()
    overlay(arr1, arr2, res, traits)
    return res


def trapezoid_decompose(arr):
    l = []
    trapezoid_arr = Arrangement_2()
    decompose(arr, l)
    verticals = []
    for pair in l:
        # pair is a tuple
        # pair[0] is an arrangement vertex
        # pair[1] is a pair holding the objects (vertex, halfedge, or face) above and below the vertex, that is,
        # the objects hit by the vertical walls emanating from the vertex
        v0 = pair[0]
        below_obj, upper_obj = pair[1]
        # if the feature above the previous vertex is not the current vertex,
        # add a vertical segment towards below feature
        down_curve = add_vertical_segment(arr, v0, below_obj, False)
        if down_curve is not None:
            verticals.append(down_curve)
        up_curve = add_vertical_segment(arr, v0, upper_obj, True)
        if up_curve is not None:
            verticals.append(up_curve)
    for curve in verticals:
        insert(trapezoid_arr, curve)
    arr = arr_overlay(arr, trapezoid_arr)
    return arr


def add_vertical_segment(arr, v0, obj, is_obj_above_vertex):
    seg = None
    v1 = None
    if obj.is_vertex():
        v1 = Vertex()
        obj.get_vertex(v1)
        seg = Segment_2(v0.point(), v1.point())
    elif obj.is_halfedge():
        he = Halfedge()
        obj.get_halfedge(he)
        assert he.direction() == ARR_RIGHT_TO_LEFT
        # test if ray needs to be added
        test_halfedge = he if is_obj_above_vertex else he.twin()
        if test_halfedge.face().data() == FREE_FACE:
            if compare_x(v0.point(), he.target().point()) == EQUAL:
                # same x coordinate, just connect
                v1 = he.target()
                seg = Segment_2(v0.point(), v1.point())
            else:
                # vertical project v to the segment, split and connect
                tangent = Line_2(he.source().point(), he.target().point())
                # project v0 upwards
                y_top = tangent.y_at_x(v0.point().x())
                seg = Segment_2(v0.point(), Point_2(v0.point().x(), y_top))
                # v1 = seg.target()
        else:
            pass  # Skipping - wall inside the obstacle
    if seg is not None:
        c0 = Curve_2(seg)
        return c0
    return None


# Input: face in configuration space
# Output: a list of its bounding points
def face_to_points(face: Face) -> List[Point_2]:
    points = []

    # traverse outer components of face
    if face.number_of_outer_ccbs() > 0:
        for bounding_he in face.outer_ccb():
            assert isinstance(bounding_he, Halfedge)
            # append points of bounding half edges
            new_point = bounding_he.source()
            assert isinstance(new_point, Vertex)
            points.append(new_point.point())
    return points


# Input: list of all points bounding a face
# Output: average point of face, or centroid
def get_face_midpoint(face: Face):
    face_points = face_to_points(face)
    if len(face_points) == 0:
        return None
    midpoint_x, midpoint_y = 0, 0

    # average out all points of face
    for point in face_points:
        midpoint_x += point_2_to_xy(point)[0]
        midpoint_y += point_2_to_xy(point)[1]
    return point_2_to_xy(Point_2(midpoint_x / len(face_points), midpoint_y / len(face_points)))


# Input: a halfedge in the arrangement
# Output: midpoint of halfedge
def get_halfedge_midpoint(he: Halfedge) -> Point_2:
    st = point_2_to_xy(he.source().point())
    end = point_2_to_xy(he.target().point())

    return point_2_to_xy(Point_2((st[0] + end[0]) / 2, (st[1] + end[1]) / 2))


"""
Input: arrangement of trapezoidal free space
Output: graph discretization of the free space, roadmap.
Notes:
Nodes of the graph are midpoints of every face in the free space, as well as midpoints of every halfedge.
Edges are added between nodes representing incident faces.a
"""


def build_roadmap(conf_space: Arrangement_2):
    roadmap = defaultdict(list)

    # traverse faces of the free space
    for face in conf_space.faces():
        if face.data() == FORBIDDEN_FACE:
            continue  # we only care about faces in the free space
        face_midpoint = get_face_midpoint(face)
        roadmap[face_midpoint] = []

        # traverse outer half edges of face
        for bounding_he in face.outer_ccb():
            if bounding_he.twin().face().data() == FORBIDDEN_FACE:
                continue
            # find midpoint of bounding half edge
            bounding_he_midpoint = get_halfedge_midpoint(bounding_he)
            # add graph edge from face midpoint to bounding half edge midpoint
            roadmap[face_midpoint].append(bounding_he_midpoint)
            # graph needs to be undirected, add edge in other direction
            if bounding_he_midpoint not in roadmap:
                roadmap[bounding_he_midpoint] = [face_midpoint]
            else:
                roadmap[bounding_he_midpoint].append(face_midpoint)
    return roadmap


def roadmap_bfs(src, dst, roadmap: dict, free_space: Arrangement_2) -> List[Point_2]:
    # locate features for start & end points
    point_locator = Arr_naive_point_location(free_space)
    src_feature = point_locator.locate(src)
    dst_feature = point_locator.locate(dst)
    f1, f2 = Face(), Face()
    if src_feature.is_face():
        src_feature.get_face(f1)
    else:
        pass  # no path exists, only semi-free
    if dst_feature.is_face():
        dst_feature.get_face(f2)
    else:
        pass  # only semi-free

    dst_loc, src_loc = get_face_midpoint(f1), get_face_midpoint(f2)
    if dst_loc not in roadmap or src_loc not in roadmap:
        return []  # free path does not exist

    fathers = bfs(roadmap, src_loc)

    if fathers == {}:
        return []

    path = [src]
    curr_node = dst_loc

    while curr_node != src_loc:
        path.append(curr_node)
        if curr_node not in fathers:
            return []  # a free path does not exist
        curr_node = fathers[curr_node]
    path.append(dst)

    return path


def bfs(roadmap: dict, src_loc: Point_2) -> dict:
    q = Queue()
    visited = set()
    fathers = {}
    q.put(src_loc)

    while not q.empty():
        curr_node = q.get()
        if curr_node not in roadmap:
            return {}  # a free path does not exist
        neighbors = roadmap[curr_node]

        for neighbor in neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                q.put(neighbor)
                fathers[neighbor] = curr_node
    return fathers


def djikstra(roadmap: dict, src_loc: Point_2) -> dict:
    nodes = set(roadmap.keys())
    distances = {n: float('infinity') for n in nodes}
    distances[src_loc] = 0
    pq = [(0, src_loc)]
    path = {}

    while len(pq) > 0:
        current_weight, min_node = heapq.heappop(pq)

        if current_weight > distances[min_node]:
            continue

        neighbors = roadmap[min_node]
        for n in neighbors:
            dist = squared_distance(xy_to_point_2(*min_node), xy_to_point_2(*n))
            weight = current_weight + math.sqrt(dist.to_double())
            if weight < distances[n]:
                distances[n] = weight
                heapq.heappush(pq, (weight, n))
                path[n] = min_node

    return path
