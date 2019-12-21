from arr2_epec_seg_ex import *
import math
def sample_polygons(start, end, length, res):
  epsilon = FT(1)
  x1 = start[0].exact()
  y1 = start[1].exact()
  a1 = start[2].exact()

  x2 = end[0].exact()
  y2 = end[1].exact()
  a2 = end[2].exact()

  clockwise = end[3]
  if (not clockwise and a2 < a1): a1 = a1 - Gmpq(2 * math.pi)
  if (clockwise and a2 > a1): a1 = a1 + Gmpq(2 * math.pi)

  dx = x2 - x1
  dy = y2 - y1
  dz = abs((a2 - a1).to_double())

  sample_count = int(math.sqrt(dx.to_double()**2 + dy.to_double()**2) + dz * (length.to_double() + epsilon.to_double()))//2+1
  #print(sample_count)

  r0 = Vector_2(-epsilon, epsilon)
  r1 = Vector_2(-epsilon, -epsilon)
  r2 = Vector_2(length + epsilon, -epsilon)
  r3 = Vector_2(length + epsilon, epsilon)

  for i in range(sample_count + 1):
    x = Gmpq(sample_count - i, sample_count) * x1 + Gmpq(i, sample_count) * x2
    y = Gmpq(sample_count - i, sample_count) * y1 + Gmpq(i, sample_count) * y2
    a = Gmpq(sample_count - i, sample_count) * a1 + Gmpq(i, sample_count) * a2

    p = Point_2(FT(x), FT(y))


    at = Aff_Transformation_2(Rotation(), FT(Gmpq(math.sin(a.to_double()))),
                              FT(Gmpq(math.cos(a.to_double()))), FT(Gmpq(1)))
    p0 = p + at.transform(r0)
    p1 = p + at.transform(r1)
    p2 = p + at.transform(r2)
    p3 = p + at.transform(r3)

    polygon = Polygon_2([p0, p1, p2, p3])
    res.append(polygon)


