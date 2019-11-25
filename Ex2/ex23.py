from arr2_epec_seg_ex import * #Arr_overlay_traits, Arrangement_2, Point_2
from conversions import *


def generate_path(path, robot, obstacles, destination):
    assert obstacles is not None
    assert robot is not None
    assert destination is not None
    # get polygonal objects
    obs = [tuples_list_to_polygon_2(o) for o in obstacles]
    r = tuples_list_to_polygon_2(robot)
    d = xy_to_point_2(*destination)
    # reflect robor to get -R
    reflect_r = reflect_polygon(r)
    # compute minkowski sum
    conf_obst = [minkowski_sum_2(reflect_r, ob).outer_boundary() for ob in obs]

    #conf_space = Polygon_set_2()
    #conf_space.insert_polygons(conf_obst)
    conf_space = obstacles_to_arrangement(conf_obst)
    print(conf_space)
    vertical_decompose(conf_space)
    print(conf_space)

    path.append(Point_2(300, 400))
    path.append(Point_2(300, 1000))
    path.append(Point_2(700, 1000))
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
    #decomp = Polygon_vertical_decomposition()
    decompose(arr, l)
    #decompose(arr, d)
    prev = None
    for pair in l:
        # pair is a tuple
        # pair[0] is an arrangement vertex
        # pair[1] is a pair holding the objects (vertex, halfedge, or face) above and below the vertex, that is, the objects hit by the vertical walls emanating from the vertex
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
        v1 = he.target.point()
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
        # TODO: Boost.Python.ArgumentError: Python argument types in
        #     Arrangement_2.split_edge(Arrangement_2, Halfedge, Segment_2, Segment_2)
        # did not match C++ signature:
        #     split_edge(class CGAL::Arrangement_2<class CGAL::Arr_segment_traits_2<class CGAL::Epeck>,class CGAL::Arr_extended_dcel<class CGAL::Arr_segment_traits_2<class CGAL::Epeck>,class boost::python::api::object,class boost::python::api::object,class boost::python::api::object,class CGAL::Arr_vertex_base<class CGAL::Point_2<class CGAL::Epeck> >,class CGAL::Arr_halfedge_base<class CGAL::Arr_segment_2<class CGAL::Epeck> >,class CGAL::Arr_face_base> > {lvalue}, class CGAL::Arrangement_on_surface_2<class CGAL::Arr_segment_traits_2<class CGAL::Epeck>,class CGAL::Arr_bounded_planar_topology_traits_2<class CGAL::Arr_segment_traits_2<class CGAL::Epeck>,class CGAL::Arr_extended_dcel<class CGAL::Arr_segment_traits_2<class CGAL::Epeck>,class boost::python::api::object,class boost::python::api::object,class boost::python::api::object,class CGAL::Arr_vertex_base<class CGAL::Point_2<class CGAL::Epeck> >,class CGAL::Arr_halfedge_base<class CGAL::Arr_segment_2<class CGAL::Epeck> >,class CGAL::Arr_face_base> > >::Halfedge {lvalue}, class CGAL::Arr_segment_2<class CGAL::Epeck> {lvalue}, class CGAL::Arr_segment_2<class CGAL::Epeck> {lvalue})
        arr.split_edge(he, source_half, target_half)
        v1 = he.target()
  else:  # obj is a face
      f = Face()
      obj.get_face(f)
      print(f)
  if seg is not None and v1 is not None:
    arr.insert_at_vertices(seg, v0, v1)
