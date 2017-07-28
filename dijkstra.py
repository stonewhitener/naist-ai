import collections
import heapq
import math


class WeightedDigraph:
    def __init__(self):
        self.__vertices = set()
        self.__edges = collections.defaultdict(list)
        self.__weights = {}

    @property
    def vertices(self):
        return self.__vertices

    def neighbors(self, v):
        return self.__edges[v]

    def weight(self, u, v):
        return self.__weights[(u, v)]

    def add_vertex(self, label):
        self.__vertices.add(label)

    def add_edge(self, begin, end, weight):
        self.__edges[begin].append(end)
        self.__weights[(begin, end)] = weight


def dijkstra(graph, start, goal=None):
    frontier = []
    explored = set()

    dist = dict.fromkeys(graph.vertices, math.inf)
    prev = dict.fromkeys(graph.vertices, None)

    heapq.heappush(frontier, (0, start))
    dist[start] = 0

    while len(frontier) != 0:
        _, v = heapq.heappop(frontier)
        explored.add(v)

        if v == goal:
            return dist, prev

        for u in graph.neighbors(v):
            cost = dist[v] + graph.weight(v, u)
            if u not in explored and u not in [_[1] for _ in frontier]:
                prev[u] = v
                dist[u] = cost
                heapq.heappush(frontier, (cost, u))
            elif u in [_[1] for _ in frontier]:
                if cost < dist[u]:
                    prev[u] = v
                    dist[u] = cost

    return dist, prev


def shortest_path(graph, start, goal):
    _, prev = dijkstra(graph, start, goal)

    path = []
    vertex = goal

    while vertex is not None:
        path.append(vertex)
        vertex = prev[vertex]

    return list(reversed(path))


def main():
    g = WeightedDigraph()

    g.add_vertex('s')
    g.add_vertex('x')
    g.add_vertex('y')
    g.add_vertex('z')
    g.add_vertex('w')
    g.add_vertex('r')
    g.add_vertex('t')

    g.add_edge('s', 'x', 3)
    g.add_edge('s', 'y', 5)
    g.add_edge('x', 'z', 8)
    g.add_edge('y', 'x', 1)
    g.add_edge('y', 'z', 4)
    g.add_edge('y', 'w', 2)
    g.add_edge('y', 'r', 10)
    g.add_edge('z', 't', 4)
    g.add_edge('w', 'z', 1)
    g.add_edge('r', 't', 4)

    print(shortest_path(g, 's', 't'))


if __name__ == '__main__':
    main()
