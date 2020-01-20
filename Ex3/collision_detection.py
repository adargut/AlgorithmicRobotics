from arr2_epec_seg_ex import *
import math


def is_position_valid(x, y, a, l, polygon_list, epsilon):
    # obtain a segment representing the robot
    if epsilon == FT(Gmpq(0)):
        v = Vector_2(Point_2(0, 0), Point_2(l, FT(Gmpq(0))))
        at = Aff_Transformation_2(Rotation(), FT(Gmpq(math.sin(a.to_double()))),
                                  FT(Gmpq(math.cos(a.to_double()))), FT(Gmpq(1)))
        v = at.transform(v)
        s = Segment_2(Point_2(x, y), Point_2(x, y) + v)

        # check intersection with each obstacle
        for polygon in polygon_list:
            assert (isinstance(polygon, Polygon_2))
            if polygon.has_on_bounded_side(s.source()) or polygon.has_on_bounded_side(s.target()):
                return False
            for edge in polygon.edges():
                if not intersection(edge, s).empty(): return False
        return True

    else:
        # transform the robot into a thin rectangle (of width epsilon and length l+epsilon)
        r0 = Vector_2(-epsilon, epsilon)
        r1 = Vector_2(-epsilon, -epsilon)
        r2 = Vector_2(l + epsilon, -epsilon)
        r3 = Vector_2(l + epsilon, epsilon)
        p = Point_2(x, y)

        at = Aff_Transformation_2(Rotation(), FT(Gmpq(math.sin(a.to_double()))),
                                  FT(Gmpq(math.cos(a.to_double()))), FT(Gmpq(1)))
        p0 = p + at.transform(r0)
        p1 = p + at.transform(r1)
        p2 = p + at.transform(r2)
        p3 = p + at.transform(r3)

        rectangle = Polygon_2([p0, p1, p2, p3])

        # check intersection with each obstacle
        for polygon in polygon_list:
            assert (isinstance(polygon, Polygon_2))
            for p in rectangle.vertices():
                if polygon.has_on_bounded_side(p): return False
            for p in polygon.vertices():
                if rectangle.has_on_bounded_side(p): return False
            for edge1 in polygon.edges():
                for edge2 in rectangle.edges():
                    if not intersection(edge1, edge2).empty(): return False
        return True


if __name__ == "__main__":
    poly = Polygon_2([Point_2(0, 0), Point_2(1, 0), Point_2(1, 1)])
    x = FT(0)
    y = FT(-1)
    a = FT(1)
    l = FT(1)
    eps = FT(0)
    print(is_position_valid(x, y, a, l, [poly], eps))
