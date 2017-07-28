import collections
import heapq

import math


class Graph:
    def __init__(self):
        self.vertices = set()
        self.edges = collections.defaultdict(list)
        self.weights = {}

    def neighbors(self, v):
        return self.edges[v]

    def weight(self, v1, v2):
        return self.weights[v1, v2]

    def add_vertex(self, label):
        self.vertices.add(label)

    def add_edge(self, begin, end, weight):
        self.edges[begin].append(end)
        self.weights[(begin, end)] = weight


def astar(graph, h, start, goal=None):
    frontier = []
    explored = set()
    dist = dict.fromkeys(graph.vertices, math.inf)
    esti = dict.fromkeys(graph.vertices, math.inf)
    prev = dict.fromkeys(graph.vertices, None)

    dist[start] = 0
    esti[start] = dist[start] + h(start)
    heapq.heappush(frontier, (dist[start] + h(start), start))

    while len(frontier) != 0:
        _, v = heapq.heappop(frontier)
        print(f"Expand: {v}")
        explored.add(v)

        if v == goal:
            return dist, prev

        for u in graph.neighbors(v):
            cost = dist[v] + graph.weight(v, u)
            if u not in [_[1] for _ in frontier] and u not in explored:
                dist[u] = cost
                esti[u] = dist[u] + h(u)
                prev[u] = v
                heapq.heappush(frontier, (esti[u], u))
            elif u in [_[1] for _ in frontier]:
                if cost < dist[u]:
                    dist[u] = cost
                    esti[u] = dist[u] + h(u)
                    prev[u] = v
            elif u in explored:
                if cost < dist[u]:
                    dist[u] = cost
                    esti[u] = dist[u] + h(u)
                    prev[u] = v
                    explored.remove(u)
                    heapq.heappush(frontier, (esti[u], u))
            print(f"h({v}) <= c({v}, {u}) + h({u}) ({h(v)} <= {graph.weight(v, u)} + {h(u)}, {h(v) <= graph.weight(v, u) + h(u)})")

    return dist, prev


def shortest_path(graph, h, start, goal):
    _, prev = astar(graph, h, start, goal)

    path = []
    vertex = goal

    while vertex is not None:
        path.append(vertex)
        vertex = prev[vertex]

    return list(reversed(path))


def main():
    g = Graph()

    g.add_vertex('s')
    g.add_vertex('x')
    g.add_vertex('y')
    g.add_vertex('z')
    g.add_vertex('t')

    g.add_edge('s', 'x', 1)
    g.add_edge('s', 'y', 5)
    g.add_edge('s', 't', 9)
    g.add_edge('x', 'z', 1)
    g.add_edge('x', 't', 6)
    g.add_edge('y', 'z', 1)
    g.add_edge('y', 't', 2)
    g.add_edge('z', 'y', 2)
    g.add_edge('z', 't', 6)

    h = lambda label: 2 if label == 's' \
        else 1 if label == 'x' \
        else 2 if label == 'y' \
        else 3 if label == 'z' \
        else 0 if label == 't' \
        else math.inf

    print(f"Shortest path: {shortest_path(g, h, 's', 't')}")


if __name__ == '__main__':
    main()
