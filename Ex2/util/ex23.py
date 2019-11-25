from arr2_epec_seg_ex import Point_2


def generate_path(path, robot, obstacles, destination):
    path.append(Point_2(300, 400))
    path.append(Point_2(300, 1000))
    path.append(Point_2(700, 1000))
    print(path)
    pass


generate_path([], 0, 0, 0)
