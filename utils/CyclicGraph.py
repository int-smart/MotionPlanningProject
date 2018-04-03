# Python program to detect cycle
# in a graph

from collections import defaultdict
from copy import deepcopy


class CyclicGraph():
    def __init__(self, vertices):
        self.graph = defaultdict(list)              #This is adjacency list representation. This has vertices pointing towards the vertices its connected to.
        self.V = vertices                           #Total vertices in the form of list

    def addEdge(self, u, v):
        self.graph[u].append(v)

    def removeEdge(self, u, v):
        self.graph[u].remove(v)

    def isCyclicUtil(self, v, visited, recStack):

        # Mark current node as visited and
        # adds to recursion stack
        visited[v] = True
        recStack[v] = True

        # Recur for all neighbours
        # if any neighbour is visited and in
        # recStack then graph is cyclic
        for neighbour in self.graph[v]:
            if visited[neighbour] == False:
                if self.isCyclicUtil(neighbour, visited, recStack) == True:
                    return True
            elif recStack[neighbour] == True:
                return True

        # The node needs to be poped from
        # recursion stack before function ends
        recStack[v] = False
        return False

    # Returns true if graph is cyclic else false
    def isCyclic(self):
        visited = [False] * self.V
        recStack = [False] * self.V
        for node in range(self.V):
            if visited[node] == False:
                if self.isCyclicUtil(node, visited, recStack) == True:
                    return True
        return False

    @staticmethod
    def getStartNodes(graph, vertexTotal):
        visited = [False] * vertexTotal
        for queryVertex in graph.keys():
            for connectedVertex in graph[queryVertex]:
                visited[connectedVertex] = True
        return visited
#     Returns the topological ordering of vertices
    @staticmethod
    def tSort(graph):
        adjList = deepcopy(graph.graph)
        sortedList = []
        visited = CyclicGraph.getStartNodes(adjList, graph.V)
        startNodes = [x for x in range(len(visited)) if visited[x] == False]
        while len(startNodes) != 0:
            nodeIndex = startNodes.pop()
            sortedList.append(nodeIndex)
            for neighbor in graph.graph[nodeIndex]:
                adjList[nodeIndex].remove(neighbor)
                newVisitedSet = CyclicGraph.getStartNodes(adjList, graph.V)
                if newVisitedSet[neighbor] == False:
                    startNodes.append(neighbor)
        return sortedList

if __name__=="__main__":
    g = Graph(4)
    g.addEdge(0, 1)
    g.addEdge(0, 2)
    g.addEdge(1, 2)
    g.addEdge(2, 0)
    g.addEdge(2, 3)
    g.addEdge(3, 3)
    if g.isCyclic() == 1:
        print "Graph has a cycle"
    else:
        print "Graph has no cycle"
