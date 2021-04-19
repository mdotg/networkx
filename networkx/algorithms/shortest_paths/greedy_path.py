import networkx as nx
from heapq import heappush, heappop
from networkx.algorithms.shortest_paths.weighted import _weight_function
from itertools import count

__all__ = [
    "greedy_path",
]

def greedy_path(G, source, target, heuristic=None, weight="weight"):

    if source not in G or target not in G:
        msg = f"Either source {source} or target {target} is not in G"
        raise nx.NodeNotFound(msg)

    if heuristic is None:
        # Heuristic h = 0 -> same as Dijkstra's algorithm
        return nx.dijkstra_path(G, source, target)

    push = heappush
    pop = heappop
    weight = _weight_function(G, weight)
    c = count()
    queue = [(0, next(c), source, 0, None)]

    enqueued = {}
    explored = {}

    while queue:
    
        hq, __, curnode, dist, parent = pop(queue)

        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path, dist

        if curnode in explored:
            if explored[curnode] is None:
                continue
                
            _, h = enqueued[curnode]
            if h < hq:
                continue
        
        explored[curnode] = parent
        
        for neighbor, w in G[curnode].items():
            ncost = dist + weight(curnode, neighbor, w)
            if neighbor in enqueued:
                _, h = enqueued[neighbor]
                
                if h <= heuristic(neighbor):
                    continue
            else:
                h = heuristic(neighbor)
            
            enqueued[neighbor] = ncost, h

            push(queue, (h, next(c), neighbor, ncost, curnode))

    raise nx.NetworkXNoPath(f"Node {target} not reachable from {source}")
    