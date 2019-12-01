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
    r = tuples_list_to_polygon_2(robot)
    d = xy_to_point_2(*destination)
    source = (r.top_vertex())

    conf_space = compute_configuration_space(obs, r, d)
    # print(conf_space)
    # compute trapezoid decomposition of conf_space
    trapezoid_space = trapezoid_decompose(conf_space)
    print(trapezoid_space)
    # build roadmap
    print("Performing roadmap")
    roadmap = build_roadmap(trapezoid_space)
    print("\tDone building roadmap")
    # bfs on roadmap
    print("Started bfs on roadmap")
    roadmap_path = roadmap_bfs(source, d, roadmap, trapezoid_space)
    for i in range(1, len(roadmap_path) - 1):
        roadmap_path[i] = xy_to_point_2(roadmap_path[i][0], roadmap_path[i][1])
    print(roadmap_path)
    print("Done bfsing")
    print("bfsing from to")
    print(source, d)
    for vertex in trapezoid_space.vertices():
        path.append(vertex.point())
    return [e.curve() for e in trapezoid_space.halfedges()]


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


def compute_configuration_space(obstacles: List[Polygon_2], robot: Polygon_2, destination: Point_2) -> Arrangement_2:
    # move robot top to (0, 0)
    top_x, top_y = point_2_to_xy(robot.top_vertex())
    moved_robot_points = [(x - top_x, y - top_y) for (x, y) in polygon_2_to_tuples_list(robot)]
    start_robot = tuples_list_to_polygon_2(moved_robot_points)
    reflected_robot = reflect_polygon(start_robot)
    arr = Arrangement_2()

    for obstacle in obstacles:
        # create obstacle
        obs_arr = Arrangement_2()
        msum = minkowski_sum_2(reflected_robot, obstacle)
        boundary = msum.outer_boundary()
        assert isinstance(boundary, Polygon_2)
        insert_non_intersecting_curves(obs_arr, list(map(Curve_2, boundary.edges())))
        # set up face data for obstacle
        obs_halfedge = list(obs_arr.edges())[0]
        f = obs_halfedge.face()
        assert isinstance(f, Face)
        if f.is_unbounded():
            f.set_data(FREE_FACE)
            obs_halfedge.twin().face().set_data(FORBIDDEN_FACE)
        else:
            f.set_data(FORBIDDEN_FACE)
            obs_halfedge.twin().face().set_data(FREE_FACE)

        # overlay the new obstacle over the previous data
        arr = arr_overlay(arr, obs_arr)
        print(arr)

    # Bounding Box
    bbox_arr = Arrangement_2()
    robot_bbox = robot.bbox()
    assert isinstance(robot_bbox, Bbox_2)

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
    bbox_halfedge = list(bbox_arr.edges())[0]
    f = bbox_halfedge.face()
    assert isinstance(f, Face)
    if f.is_unbounded():
        f.set_data(FORBIDDEN_FACE)
        bbox_halfedge.twin().face().set_data(FREE_FACE)
    else:
        f.set_data(FREE_FACE)
        bbox_halfedge.twin().face().set_data(FORBIDDEN_FACE)
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
    decompose(arr, l)
    verticals = []
    for pair in l:
        # pair is a tuple
        # pair[0] is an arrangement vertex
        # pair[1] is a pair holding the objects (vertex, halfedge, or face) above and below the vertex, that is,
        # the objects hit by the vertical walls emanating from the vertex
        v0 = pair[0]
        print(f"Now decomposing {v0.point()}")
        below_obj, upper_obj = pair[1]
        # if the feature above the previous vertex is not the current vertex,
        # add a vertical segment towards below feature
        print("\tBelow")
        # TODO: restore the pair and check with previous vertex that I removed
        down_curve = add_vertical_segment(arr, v0, below_obj, False)
        if down_curve is not None:
            verticals.append(down_curve)
        print("\tAbove")
        up_curve = add_vertical_segment(arr, v0, upper_obj, True)
        if up_curve is not None:
            verticals.append(up_curve)
    print("Done decomposing")
    # res_arr = arr_overlay(arr, verticals)
    for curve in verticals:
        insert(arr, curve)
    return arr


def add_vertical_segment(arr, v0, obj, is_obj_above_vertex):
    seg = None
    v1 = None
    if obj.is_vertex():
        v1 = Vertex()
        obj.get_vertex(v1)
        print(f"\tVertex - {v1.point()}")
        seg = Segment_2(v0.point(), v1.point())
    elif obj.is_halfedge():
        he = Halfedge()
        obj.get_halfedge(he)
        print(f"\tHalfedge - {he.curve()}")
        assert he.direction() == ARR_RIGHT_TO_LEFT
        # test if ray needs to be added
        test_halfedge = he if is_obj_above_vertex else he.twin()
        if test_halfedge.face().data() == FREE_FACE:
            print("\t\tCalculating vertical wall")
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
            print("\t\tSkipping - wall inside the obstacle")
    if seg is not None:
        c0 = Curve_2(seg)
        print(f'\t\tAdding {c0}')
        return c0
    return None


"""
Input: arrangement of trapezoidal free space
Output: graph discretization of the free space, roadmap.
Notes:
Nodes of the graph are midpoints of every face in the free space, as well as midpoints of every halfedge.
Edges are added between nodes representing incident faces.a
"""


# Input: face in configuration space
# Output: a list of its bounding points
def face_to_points(face: Face) -> List[Point_2]:
    points = []

    # traverse outer components of face
    for bounding_he in face.outer_ccb():
        assert isinstance(bounding_he, Halfedge)
        # append points of bounding half edges
        new_point = bounding_he.source()
        assert isinstance(new_point, Vertex)
        points.append(new_point.point())
    assert len(points) != 0
    return points


# Input: list of all points bounding a face
# Output: average point of face, or centroid
def get_face_midpoint(face: Face):
    face_points = face_to_points(face)
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


# TODO Remove assertions before submitting
def build_roadmap(conf_space: Arrangement_2):
    roadmap = defaultdict(list)

    # traverse faces of the free space
    for face in conf_space.faces():
        if face.data() == FORBIDDEN_FACE or face.is_unbounded():  # TODO: USE CONST
            continue  # we only care about faces in the free space
        face_midpoint = get_face_midpoint(face)
        roadmap[face_midpoint] = []

        # traverse outer half edges of face
        for bounding_he in face.outer_ccb():
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


def roadmap_bfs(src, dst, roadmap: defaultdict(list), free_space: Arrangement_2) -> List[Point_2]:
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
    assert dst_loc in roadmap and src_loc in roadmap

    q = Queue()
    visited = set()
    fathers = {}
    q.put(src_loc)

    while not q.empty():
        print("\tQueue is now", q.queue)
        curr_node = q.get()
        neighbors = roadmap[curr_node]

        for neighbor in neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                q.put(neighbor)
                fathers[neighbor] = curr_node

    path = [src]
    curr_node = dst_loc

    while curr_node != src_loc:
        path.append(curr_node)
        curr_node = fathers[curr_node]
    path.append(dst)

    return path


"""
TODO:
Use Arr_naive_point_location
get where d and top are for start and finish
BFS from start to finsih
add paths for "getting on the roadmap"
yasss
"""
