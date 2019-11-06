# noinspection PyUnresolvedReferences
from oskarSolver import CubeGraph
# noinspection PyUnresolvedReferences
from oskarSolver import OskarSolver

class OskarTester:
    cubeGraph = CubeGraph()
    oskarSolver = OskarSolver()

    def testCube(self, cubeGraph=cubeGraph):
        # Testing addVertex method
        cubeGraph.addVertex((0, 1, 2))
        cubeGraph.addVertex((0, 1, 3))
        cubeGraph.addVertex((0, 1, 4))

        print(cubeGraph.vertices)
        print((0, 1, 2) in cubeGraph.vertices)
        print((0, 1, 8) in cubeGraph.vertices)

        # Testing addEdges method
        cubeGraph.addEdge(0, 1)
        cubeGraph.addEdge(1, 2)
        print(cubeGraph.graph)

    def testSolver(self, oskarSolver=oskarSolver):
        def testFormatInput():
            path = "util/oskar_input.txt"

            res = oskarSolver.getInput(path)
            print(res[0])
            print(res[1])
            print(res[2])

            return res

        def testBuildGraph():
            matrices = oskarSolver.getInput("util/oskar_input.txt")
            oskarSolver.buildGraph(matrices)

        def testFindSolution():
            testBuildGraph()
            oskarSolver.findSolution((1, 7, 9), (5, 1, 1))

        def testBuildSolution():
            testFindSolution()
            sol = oskarSolver.buildSolution((1, 7, 9), (5, 1, 1))

        def testParseSolution():
            testBuildSolution()
            sol = oskarSolver.buildSolution((1, 7, 9), (5, 1, 1))
            sol = oskarSolver.parseSolution(sol)
            print("got solution")
            print(sol)

            formatted = oskarSolver.formatSolution(sol)
            print("formatted output path")
            print(formatted)

        # testFormatInput()
        # testBuildGraph()
        # testFindSolution()
        # testBuildSolution()
        testParseSolution()

    def runTests(self):
        # self.testCube()
        self.testSolver()


def main():
    oskarTester = OskarTester()

    # Run tests
    oskarTester.runTests()


if __name__ == '__main__':
    main()
