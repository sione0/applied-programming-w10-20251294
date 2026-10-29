"""Supplied graph contracts and parent-chain reconstruction."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SearchResult:
    path: tuple | None
    cost: float
    expanded: tuple


def validate_search(graph, start, goal, heuristic, edge_is_valid):
    if start not in graph or goal not in graph:
        raise ValueError("start and goal must be graph keys")
    if not callable(heuristic) or not callable(edge_is_valid):
        raise TypeError("heuristic and edge_is_valid must be callable")
    for node, edges in graph.items():
        h = float(heuristic(node))
        if not math.isfinite(h) or h < 0:
            raise ValueError("heuristic values must be finite and nonnegative")
        for neighbor, cost in edges:
            if neighbor not in graph:
                raise ValueError("each neighbor must be a graph key")
            if not math.isfinite(float(cost)) or cost < 0:
                raise ValueError("edge costs must be finite and nonnegative")
    if float(heuristic(goal)) != 0:
        raise ValueError("heuristic(goal) must be zero")


def reconstruct(parent, goal):
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    return tuple(reversed(path))


def all_edges_valid(left, right):
    return True

