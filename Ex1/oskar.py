from collections import defaultdict
from queue import Queue
import argparse


class CubeGraph:
    def __init__(self):
        self.graph = defaultdict(list)
        self.vertices = {}
        self.fathers = {}
        self.vertexCount = 0

    def addVertex(self, vertex):
        if vertex in self.vertices:
            return

        vertexMapping = self.vertexCount
        self.vertices[vertex] = vertexMapping
        self.vertexCount += 1

    def addEdge(self, u, v):
        self.graph[u].append(v)
        self.graph[v].append(u)

    def setFather(self, node, father):
        self.fathers[node] = father

    def getFather(self, node):
        return self.fathers[node]


class OskarSolver:
    cubeGraph = CubeGraph()

    def getInput(self, path):
        mat1, mat2, mat3 = [], [], []
        currMat = mat1
        counter = 0

        with open(path) as inputFile:
            for line in inputFile:
                if counter == 0:
                    counter += 1
                    continue
                if line.isspace():
                    if id(currMat) == id(mat1):
                        currMat = mat2
                    else:
                        currMat = mat3
                else:
                    addedLine = []
                    for char in line:
                        if not char.isspace():
                            addedLine.append(char)
                    currMat.append(addedLine)

        return [mat1, mat2, mat3]

    def buildGraph(self, matrices):
        xDim, yDim, zDim = 11, 11, 11
        mat1, mat2, mat3 = matrices[0], matrices[1], matrices[2]

        def legalVertex(vertex):
            x, y, z = vertex[0], vertex[1], vertex[2]

            if mat1[y][x] == '0' and mat2[z][y] == '0' and mat3[x][z] == '0':
                return True
            return False

        for i in range(xDim):
            for j in range(yDim):
                for k in range(zDim):
                    vertex = (i, j, k)

                    if legalVertex(vertex):
                        self.cubeGraph.addVertex(vertex)

        for vertex in self.cubeGraph.vertices.keys():
            x, y, z = vertex[0], vertex[1], vertex[2]
            neighbors = [(x - 1, y, z), (x, y - 1, z), (x, y, z - 1),
                         (x + 1, y, z), (x, y + 1, z), (x, y, z + 1)]

            for neighbor in neighbors:
                if neighbor in self.cubeGraph.vertices:
                    u, v = self.cubeGraph.vertices[vertex], self.cubeGraph.vertices[neighbor]
                    self.cubeGraph.addEdge(u, v)

    def findSolution(self, start, end):
        currNode, vertexQueue = start, Queue()
        visited = set()

        vertexQueue.put(self.cubeGraph.vertices[currNode])
        visited.add(self.cubeGraph.vertices[currNode])
        while not vertexQueue.empty():
            node = vertexQueue.get()
            neighbors = self.cubeGraph.graph[node]

            for neighbor in neighbors:
                if neighbor not in visited:
                    self.cubeGraph.setFather(neighbor, node)
                    visited.add(neighbor)
                    vertexQueue.put(neighbor)

                    # Found path to end node
                    if neighbor == self.cubeGraph.vertices[end]:
                        return end

    def buildSolution(self, start, end):
        start, end = self.cubeGraph.vertices[start], self.cubeGraph.vertices[end]
        solution, currNode = [end], end

        # Solution path is built in this logic: [end, father(end), father(father(end)),...,start]
        while currNode != start:
            currNode = self.cubeGraph.getFather(currNode)
            solution.append(currNode)

        # Solution path was built in a reverse manner, so we must reverse it
        def reverseSolution():
            util_arr = []

            while solution:
                node = solution.pop()
                util_arr.append(node)
            return util_arr

        solution = reverseSolution()
        return solution

    def parseSolution(self, solution):
        parsed_solution = []

        def _parseSolution(node):
            for idx in self.cubeGraph.vertices:
                if self.cubeGraph.vertices[idx] == node:
                    return idx

        for val in solution:
            parsed_solution.append(_parseSolution(val))
        return parsed_solution

    def parseMove(self, moveFrom, moveTo):
        x1, y1, z1, = moveFrom[0], moveFrom[1], moveFrom[2]
        x2, y2, z2 = moveTo[0], moveTo[1], moveTo[2]

        # Moving one unit along the x-axis
        if x1 != x2:
            if x1 < x2:
                return 0
            return 1

        # Moving one unit along the y-axis
        if y1 != y2:
            if y1 < y2:
                return 2
            return 3

        # Moving one unit along the z-axis
        if z1 != z2:
            if z1 < z2:
                return 4
            return 5

    def formatSolution(self, solution):
        moveFrom = solution[0]
        moveTo = solution[1]
        currPos, formatSolution = 1, [self.parseMove(moveFrom=moveFrom, moveTo=moveTo)]

        while currPos < len(solution) - 1:
            moveFrom = solution[currPos]
            moveTo = solution[currPos + 1]
            move = self.parseMove(moveFrom=moveFrom, moveTo=moveTo)

            formatSolution.append(move)
            currPos += 1

        return formatSolution

    def getSolution(self, start, end):
        # Find a solution
        self.findSolution(start=start, end=end)
        # Recover solution from Fathers structure
        solution = self.buildSolution(start=start, end=end)
        # Parse the solution as a list of moves
        solution = self.parseSolution(solution=solution)
        # Format the list of moves as required (0 to positive along x-axis, etc)
        solution = self.formatSolution(solution=solution)

        return solution

    def solveCube(self, src, dst, filename):
        # Get matrices from input file
        matrices = self.getInput(path=filename)

        # Build graph corresponding to obstacles
        self.buildGraph(matrices=matrices)

        # Get solution for path between the input coordinates
        solution = self.getSolution(start=src, end=dst)

        return solution


def main():
    # Command line arguments for program
    ap = argparse.ArgumentParser()
    ap.add_argument("--sx", default=None, type=int, help="Source x coordinate")
    ap.add_argument("--sy", default=None, type=int, help="Source y coordinate")
    ap.add_argument("--sz", default=None, type=int, help="Source z coordinate")
    ap.add_argument("--dx", default=None, type=int, help="Destination x coordinate")
    ap.add_argument("--dy", default=None, type=int, help="Source y coordinate")
    ap.add_argument("--dz", default=None, type=int, help="Source z coordinate")
    ap.add_argument("--f", default=None, type=str, help="Filename of obstacles description")
    args = vars(ap.parse_args())

    src = (args["sx"], args["sy"], args["sz"])
    dst = (args["dx"], args["dy"], args["sz"])
    filename = args["f"]
    solver = OskarSolver()

    matrices = solver.getInput(path=filename)
    solver.buildGraph(matrices=matrices)
    solution = solver.getSolution(start=src, end=dst)

    for scalar in solution:
        print(scalar, "", end="")


if __name__ == '__main__':
    main()
