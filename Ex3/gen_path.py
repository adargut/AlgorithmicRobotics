from arr2_epec_seg_ex import *

def generate_path(path, length, obstacles, origin, destination):
  print(path)
  print(length)
  print(obstacles)
  print(origin)
  print(destination)
  path.append((FT(Gmpq(200)), FT(Gmpq(500)), FT(Gmpq("0/3")), True))
  path.append((FT(Gmpq(300)), FT(Gmpq(1000)), FT(Gmpq("2/1")), True))
  path.append((FT(Gmpq(300)), FT(Gmpq(1000)), FT(Gmpq("1/1")), False))