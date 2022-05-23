# from typing import *
import heapq
import math

import networkx as nx

# from icecream import ic

# Simple Prims algorithm

""" Prim's Minium Spanning Tree
Given an undirected weighted graph , it returns the spanning tree using Prim's algorithm and
using the Networkx library.
Parameters:
----------
input_graph: [(u,v,w),....(un,vn,wn)]
   A list of  ordered 3-tuple which represent the tree.
   Each tuple of the  in the list (u,v,w) is represents an edge from u to v in the graph
   with weight w. Vertices can either  any hashable type such as int or string
root: An optional parameter allows the call to specify a root node
returns A Networkx graph representing a Minium Cost Spanning Tree (MST) of the input Graph l

Example:
----------
 Input:
 input_graph = [('A', 'B', 2), ('B', 'D', 2), ('A', 'D', 1), ('C', 'D', 3)]
 root = 'B'
 returns: Networkx Graph representing the MST"""


# Simple Prims algorithm
def prim_find_span(root, input_graph: list) -> nx.Graph:
    """ Prim's Minium Spanning Tree
Given an undirected weighted graph , it returns the spanning tree using Prim's algorithm and
using the Networkx library.
Parameters:
----------
input_graph: [(u,v,w),....(un,vn,wn)]
A list of  ordered 3-tuple which represent the tree.
Each tuple of the  in the list (u,v,w) is represents an edge from u to v in the graph
with weight w. Vertices can either  any hashable type such as int or string
root: An optional parameter allows the call to specify a root node
returns A Networkx graph representing a Minium Cost Spanning Tree (MST) of the input Graph l

Returns:
----------
returns: Networkx Graph representing the MST

Example:
----------
 Input:
 input_graph = [('A', 'B', 2), ('B', 'D', 2), ('A', 'D', 1), ('C', 'D', 3)]
 root = 'B'
 F is the Networkx Graph  [[('B', 'A'), ('A', 'D'), ('D', 'C')]]"""
    weights = {}
    F = nx.Graph()
    heap = []
    E = {}
    vertice_set = set()
    G = nx.Graph()
    for u, v, cost in input_graph:
        G.add_edge(u, v, cost=cost)
        # ic(u, v, cost)
        vertice_set.add(u)
        vertice_set.add(v)
    # Turn heap  into priority queue
    for vertice in vertice_set:
        if vertice != root:
            heap.append((math.inf, vertice))
        weights[vertice] = math.inf
    # ic(weights)
    heapq.heapify(heap)
    # ic(heap)

    while len(heap) > 0:
        if root is not None:
            cost, new_vertice = (math.inf, root)
            root = None
        else:
            cost, new_vertice = heapq.heappop(heap)

        F.add_node(new_vertice)
        # ic(cost, new_vertice)
        if not (cost is math.inf):
            # print('Adding edge')
            F.add_edge(E[new_vertice][0], E[new_vertice][1], weight=weights[new_vertice])
        # print('neighbors =', list(G.neighbors(new_vertice)))
        heapify = False

        for v in G.neighbors(new_vertice):
            if v not in F.nodes:

                cost_new_edge = G.get_edge_data(new_vertice, v)['cost']

                if cost_new_edge < weights[v]:
                    heapify = True
                    n1 = min(v, new_vertice)
                    n2 = max(v, new_vertice)
                    # ic(heap)
                    index = heap.index((weights[v], v,))
                    # ic(heap)
                    heap[index] = (cost_new_edge, v)
                    weights[v] = cost_new_edge
                    E[v] = (n1, n2)
        if heapify:
            heapq.heapify(heap)
    return F


def read_MST(file_name: str) -> (int, list):
    with open(file_name) as in_file:
        num_nodes, num_edges = in_file.readline().split()
        num_edges = int(num_edges)
        edges = []
        for i in range(num_edges):
            u, v, w = in_file.readline().split()
            edges.append([int(u), int(v), int(w)])
        root = int(in_file.readline())
    return root, edges


def main():
    READ_FILE = False
    ##TEST_GRAPH_1 = [(1, 2, 5), (1, 3, 7)]
    TEST_GRAPH_2 = [('A', 'B', 2), ('B', 'D', 2), ('A', 'D', 1), ('C', 'D', 3)]
    if READ_FILE:
        root, edges = read_MST('MST.dat')
    else:
        root = 'B'
        edges = TEST_GRAPH_2
    # ic(root)
    # ic(edges)
    F = prim_find_span(root, edges)
    cost = 0
    for (u, v, d) in F.edges(data=True):
        cost += d['weight']
    print(f'total cost ={cost}')
    print(f'Edges in spanning tree {list(F.edges)}')
    # ic(list(F.edges))


if __name__ == '__main__':
    main()
