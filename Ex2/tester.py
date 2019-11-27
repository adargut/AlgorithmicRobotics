# some messing around to get a grasp of CGAL
from ex23 import *
from arr2_epec_seg_ex import *
d = defaultdict(list)

x = Point_2(100, 300)
y = Point_2(400, 500)
d[x] = [x, y]
d[y] = []

print(d)
x = point_2_to_xy(x)
y = point_2_to_xy(y)

x_coord = x[0] + y[0]
y_coord = x[1] + y[1]

print(x_coord, y_coord)

x = xy_to_point_2(x[0], x[1])
y = xy_to_point_2(y[0], y[1])

z = Segment_2(x, y)
print(z)