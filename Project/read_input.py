from arr2_epec_seg_ex import FT, Gmpq, Point_2


def read_polygon(filename):
    with open(filename, "r") as f:
        input_data = f.readline().split(" ")
    output = []
    for i in range(0, int(input_data[0])):
        output.append((int(input_data[2 * i + 1]), int(input_data[2 * i + 2])))
    return output


def read_disc(filename):
    with open(filename, "r") as f:
        input_data = f.readline().split(" ")
        out = [int(i) for i in input_data]
    return out


def read_point(filename):
    with open(filename, "r") as f:
        input_data = f.readline().split(" ")
        out = [int(i) for i in input_data]
    return out


def read_polygon_scene(filename):
    out = []
    with open(filename, "r") as f:
        for line in f:
            input_data = line.split(" ")
            if len(input_data) == 2:
                out.append(Point_2(FT(Gmpq(input_data[0])), FT(Gmpq(input_data[1]))))
            else:
                polygon = []
                for i in range(0, int(input_data[0])):
                    polygon.append(Point_2(FT(Gmpq(input_data[2 * i + 1])), FT((Gmpq(input_data[2 * i + 2])))))
                out.append(polygon)
    return out


def save_path(path, filename):
    file = open(filename, 'w')
    for i in range(len(path)):
        p = path[i]
        x = p.x().exact()
        y = p.y().exact()
        line = str(x.numerator()) + '/' + str(x.denominator()) + ' ' + str(y.numerator()) + '/' + str(y.denominator())
        if i < len(path) - 1:  line = line + '\n'
        file.write(line)
    file.close()


def load_path(path, filename):
    with open(filename, 'r') as file:
        for line in file:
            coords = line.split(" ")
            x0 = FT(Gmpq(coords[0]))
            y0 = FT(Gmpq((coords[1])))
            x1 = FT(Gmpq(coords[2]))
            y1 = FT(Gmpq(coords[3]))
            p0 = Point_2(x0, y0)
            p1 = Point_2(x1, y1)
            path.append((p0, p1))
