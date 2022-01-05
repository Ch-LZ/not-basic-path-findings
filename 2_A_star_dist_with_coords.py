#!/usr/bin/python3


import sys
import queue
import math
import itertools as IT
from collections import defaultdict


class AStar:
    """Bidirectional A* (a-star) algorithm aka dijkstra with potentials.
    Requires edge weights + potential function `pi` to be non-negative.

    Visits less vertexes then vanilla bidirectional dijkstra due to effect of the potential function.
    Return distance from s to t by `search` method

    Uses euclidean distance between

    Parameters
    ----------
    n : int
        number of nodes in given graph
    adj : [list[list[int]], list[list[int]]]
        TWO adjacency lists: with for given graph and for reversed graph
    cost :
        weights of edges in order given by adj list
    x : list[int] or dict[(v:int)]
        x coordinates of vertexes
    y : the same as x but y coordinates
    """
    FRW = 0
    REV = 1

    def __init__(self, n, adj, cost, x, y):
        self.n = n                  # Number of nodes
        self.inf = float('+inf')    # All distances in the graph are smaller
        self.dist = [defaultdict(lambda: self.inf),  # distances for forward
                     defaultdict(lambda: self.inf)]  # and backward searches

        self.visited = [defaultdict(lambda: False),  # if visited by forward
                        defaultdict(lambda: False)]  # and backward search
        # Graph data
        self.adj = adj
        self.cost = cost
        self.x = x
        self.y = y

        self.s = 0  # source
        self.t = 0  # target

    def pi(self, v, t):
        """Euclidean distance from v to t
        Potential of vertex v moving to the target t"""
        return math.sqrt(
            (self.x[v]-self.x[t])*(self.x[v]-self.x[t]) +
            (self.y[v]-self.y[t])*(self.y[v]-self.y[t])
        )

    def pot(self, v, side):
        """Bidirectional potential function"""
        res = (self.pi(v, self.t) - self.pi(v, self.s))/2
        res = int(res)  # to prevent effect on decimal weights of the result
        return (res if side == self.FRW else -res)

    def __clear(self):
        """Reinitialize the data structures for the next query after the previous query."""
        self.visited[0].clear()
        self.visited[1].clear()
        self.dist[0].clear()
        self.dist[1].clear()

    def visit(self, q, side, v):
        """Relax the distance to node v from direction side"""
        for nv, weight in zip(self.adj[side][v], self.cost[side][v]):
            nw = self.dist[side][v] + weight - \
                self.pot(v, side) + self.pot(nv, side)
            if self.dist[side][nv] > nw:
                self.dist[side][nv] = nw
                q[side].put((nw, nv))
        self.visited[side][v] = True

    def calc_path_len(self):
        m = float('+inf')
        shift = self.pot(self.s, self.FRW) - self.pot(self.t, self.FRW)
        for k, val in IT.chain(self.visited[self.FRW].items(),
                               self.visited[self.REV].items()):
            if val:
                cand = self.dist[self.FRW][k] + self.dist[self.REV][k]
                m = m if m < cand else cand

        return m + shift

    def query(self, s, t):
        """Return distance between vertices s and t (>=0)
        or -1 if there is no path from s to t in given graph"""
        self.__clear()
        q = [queue.PriorityQueue(), queue.PriorityQueue()]
        self.s = s
        self.t = t

        q[self.FRW].put((0, s))
        self.dist[self.FRW][s] = 0
        q[self.REV].put((0, t))
        self.dist[self.REV][t] = 0

        while not (q[self.FRW].empty() or q[self.REV].empty()):
            _, u = q[self.FRW].get()            # forward search step
            if not self.visited[self.FRW][u]:
                self.visit(q, self.FRW, u)

            if self.visited[self.REV][u]:       # forward search check
                return self.calc_path_len()

            _, u = q[self.REV].get()            # backward search step
            if not self.visited[self.REV][u]:
                self.visit(q, self.REV, u)

            if self.visited[self.FRW][u]:       # backward search check
                return self.calc_path_len()

        return -1


def readl():
    return map(int, sys.stdin.readline().split())


if __name__ == '__main__':
    """Read graph in following format:
    n m 
    x1 y1
    x2 y2
    ...
    xn yn
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
    n, m = readl()
    x = [0 for _ in range(n)]
    y = [0 for _ in range(n)]
    adj = [[[] for _ in range(n)], [[] for _ in range(n)]]
    cost = [[[] for _ in range(n)], [[] for _ in range(n)]]
    for i in range(n):
        a, b = readl()
        x[i] = a
        y[i] = b
    for e in range(m):
        u, v, c = readl()
        adj[0][u-1].append(v-1)
        cost[0][u-1].append(c)
        adj[1][v-1].append(u-1)
        cost[1][v-1].append(c)
    t, = readl()
    astar = AStar(n, adj, cost, x, y)
    for i in range(t):
        s, t = readl()
        print(astar.query(s-1, t-1))
