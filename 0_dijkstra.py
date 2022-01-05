# Uses python3

import sys
import queue
from collections import defaultdict


def dij_distance(adj, cost, s, t):
    """Search length of shortest path from s to t in given graph by Dijkstra's algorithm.
    Requires all edges to be non-negative.
    Parameters
    ----------
    adj : list[list[int]]
        adjacency list of given graph
    cost : list[list[int]]
        weights of edges in order given by adj list
    s : int
        source vertex
    t : int
        target vertex
    Returns
    ------- 
    dist[t] : int 
        distance from s to t (>=0) 
        or -1 if there is no path from s to t in given graph"""
    dist = defaultdict(lambda: float('+inf'))
    visited = defaultdict(lambda: False)
    dist[s] = 0
    heap = queue.PriorityQueue()
    for i in range(len(adj)):
        heap.put((dist[i], i))

    while not heap.empty():
        _, u = heap.get()
        if not visited[u]:
            for idx, v in enumerate(adj[u]):
                if dist[v] > dist[u] + cost[u][idx]:
                    dist[v] = dist[u] + cost[u][idx]
                    heap.put((dist[v], v))

        visited[u] = True
    if dist[t] != float('inf'):
        return dist[t]
    else:
        return -1


if __name__ == '__main__':

    """Read graph in following format:
    n m 
    v1 u1 w1
    v2 u2 w2
    ...
    vm um wm
    q
    s1 t1
    s2 t2
    ...
    sq tq

    n - number of vertexes
    m - number of edges
    xi yi - coordinates of vertex
    q - number of queries
    si ti - source and target vertexes of given query
    """
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n, m = data[0:2]
    data = data[2:]
    edges = list(
        zip(zip(data[0:(3 * m):3], data[1:(3 * m):3]), data[2:(3 * m):3]))
    data = data[3 * m:]
    adj = [[] for _ in range(n)]
    cost = [[] for _ in range(n)]
    for ((a, b), w) in edges:
        adj[a - 1].append(b - 1)
        cost[a - 1].append(w)
    s, t = data[0] - 1, data[1] - 1
    print(dij_distance(adj, cost, s, t))
