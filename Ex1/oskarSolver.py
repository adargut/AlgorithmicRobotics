from collections import defaultdict
from queue import Queue

import numpy as np


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

    def formatInput(self, path):
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

            if mat1[x][y] == '0' and mat2[y][z] == '0' and mat3[z][x] == '0':
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
            neighbors = [(x - 1, y, z), (x, y - 1, z), (x, y, z - 1)]

            for neighbor in neighbors:
                if neighbor in self.cubeGraph.vertices:
                    u, v = self.cubeGraph.vertices[vertex], self.cubeGraph.vertices[neighbor]
                    self.cubeGraph.addEdge(u, v)

    def findSolution(self, start, end):
        currNode, vertexQueue = start, Queue()
        visited, solution = set(), []

        vertexQueue.put(currNode)
        while not vertexQueue.empty():
            node = vertexQueue.get()
            neighbors = self.cubeGraph.graph[node]

            print("curr node ", node)
            print("neighbors are ", neighbors)

            for neighbor in neighbors:
                if neighbor not in visited:
                    self.cubeGraph.setFather(neighbor, node)
                    visited.add(neighbor)
                    vertexQueue.put(neighbor)

                    # Found path to end node
                    if neighbor == end:
                        return end

    def buildSolution(self, start, end):
        solution = [end]
        currNode = end

        # Solution path is built in this logic: [end, father(end), father(father(end)),...,start]
        while currNode != start:
            currNode = self.cubeGraph.getFather(currNode)
            solution.append(currNode)
            currNode = self.cubeGraph.getFather(currNode)

        # Solution path was built in a reverse manner, so we must reverse it
        def reverseSolution():
            util_arr = []

            while solution:
                node = solution.pop()
                util_arr.append(node)
            return util_arr

        reverseSolution()
        return solution

x = Queue()
x.put(5)
x.put(4)
x.get()
x.get()
print(x.empty())