from collections import defaultdict
from typing import List

from arr2_epec_seg_ex import *  # Arr_overlay_traits, Arrangement_2, Point_2
from conversions import *


def generate_path(path, robot, obstacles, destination):
    assert obstacles is not None
    assert robot is not None
    assert destination is not None
    # get polygonal objects
    obs = [tuples_list_to_polygon_2(o) for o in obstacles]
    r = tuples_list_to_polygon_2(robot)
    d = xy_to_point_2(*destination)
    # TODO add a bounding box to conf_space, maybe overlay with Bbox_2?
    conf_space = compute_configuration_space(obs, r)
    # print(conf_space)
    vertical_decompose(conf_space)
    build_roadmap(conf_space)
    # print(conf_space)

    # path.append(Point_2(300, 400))
    # path.append(Point_2(300, 1000))
    # path.append(Point_2(700, 1000))
    for vertex in conf_space.vertices():
        path.append(vertex.point())
    pass


def reflect_polygon(polygon):
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


def compute_configuration_space(obstacles: List[Polygon_2], robot: Polygon_2) -> Arrangement_2:
    reflected_robot = reflect_polygon(robot)
    arr = Arrangement_2()
    for obstacle in obstacles:
        msum = minkowski_sum_2(reflected_robot, obstacle)
        boundary = msum.outer_boundary()
        assert isinstance(boundary, Polygon_2)
        insert_non_intersecting_curves(arr, map(Curve_2, boundary.edges()))

    def mark_free_space(face: Face, is_free: bool) -> None:
        face.set_data(int(is_free))
        if face.number_of_inner_ccbs() > 0:
            for hole in face.holes():
                mark_free_space(hole, not is_free)

    unbounded_face = arr.unbounded_face()
    mark_free_space(unbounded_face, True)
    return arr


def arr_overlay(arr1, arr2):
    def first(x, y):
        return x

    def empty(x, y):
        return None

    traits = Arr_overlay_traits(empty, empty, empty, empty, empty, empty, empty, empty, empty, first)
    res = Arrangement_2()
    overlay(arr1, arr2, res, traits)
    return res


def vertical_decompose(arr):
    l = []
    # decomp = Polygon_vertical_decomposition()
    decompose(arr, l)
    # decompose(arr, d)
    prev = None
    for pair in l:
        # pair is a tuple
        # pair[0] is an arrangement vertex
        # pair[1] is a pair holding the objects (vertex, halfedge, or face) above and below the vertex, that is,
        # the objects hit by the vertical walls emanating from the vertex
        v0 = pair[0]
        print(v0)
        below, upper = pair[1]
        prev_lower_point = Point_2()
        # if the feature above the previous vertex is not the current vertex,
        # add a vertical segment towards below feature
        if prev is not None and hasattr(prev[1], 'get_point'):
            is_prev_below_obj_point = prev[1].get_point(prev_lower_point)
        else:
            is_prev_below_obj_point = False
        if prev is None or not is_prev_below_obj_point or not prev_lower_point == v0.point():
            add_vertical_segment(arr, v0, below)
        add_vertical_segment(arr, v0, upper)
        prev = pair[1]


def add_vertical_segment(arr, v0, obj):
    seg = None
    v1 = None
    if obj.is_vertex():
        v1 = Vertex()
        obj.get_vertex(v1)
        seg = Segment_2(v0.point(), v1.point())
    elif obj.is_halfedge():
        he = Halfedge()
        obj.get_halfedge(he)
        if compare_x(v0.point(), he.target().point()) == EQUAL:
            # same x coordinate, just connect
            v1 = he.target().point()
            seg = Segment_2(v0.point(), v1.point())
        else:
            # vertical project v to the segment, split and connect
            tangent = Line_2(he.source().point(), he.target().point())
            # project v0 upwards
            x, y = point_2_to_xy(v0.point())
            vertical = Line_2(v0.point(), xy_to_point_2(x, y + 1))
            # intersect
            res = intersection(tangent, vertical)
            p = Point_2()
            res.get_point(p)
            seg = Segment_2(v0.point(), p)
            source_half = Segment_2(he.source().point(), p)
            target_half = Segment_2(p, he.target().point())
            assert isinstance(arr, Arrangement_2)
            c1 = Curve_2(source_half)
            c2 = Curve_2(target_half)
            arr.split_edge(he, c1, c2)
            v1 = he.target()
    else:  # obj is a face
        f = Face()
        obj.get_face(f)
        print(f)
    if seg is not None and v1 is not None:
        c0 = Curve_2(seg)
        arr.insert_at_vertices(c0, v0, v1)


"""
Input: arrangement of trapezoidal free space
Output: graph discretization of the free space, roadmap.
Notes:
Nodes of the graph are midpoints of every face in the free space, as well as midpoints of every halfedge.
Edges are added between nodes representing incident faces.a
"""


# TODO Remove assertions before submitting
def build_roadmap(conf_space: Arrangement_2):
    roadmap = defaultdict(list)

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
    def get_face_midpoint(face: Face) -> Point_2:
        face_points = face_to_points(face)
        midpoint_x, midpoint_y = 0, 0

        # average out all points of face
        for point in face_points:
            midpoint_x += point_2_to_xy(point)[0]
            midpoint_y += point_2_to_xy(point)[1]
        return Point_2(midpoint_x / len(face_points), midpoint_y / len(face_points))

    # Input: a halfedge in the arrangement
    # Output: midpoint of halfedge
    def get_halfedge_midpoint(he: Halfedge) -> Point_2:
        x_coord = (he.source()[0] + he.twin().source()[0]) / 2
        y_coord = (he.source()[1] + he.twin().source()[1]) / 2

        return Point_2(x_coord, y_coord)

    # traverse faces of the free space
    for face in conf_space.faces():
        if face.data() != 1:
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
