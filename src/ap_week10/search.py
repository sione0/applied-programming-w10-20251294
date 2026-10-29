"""TODO 1: cost improvement and priority inside a supplied search loop."""
from heapq import heappop, heappush
from itertools import count
from .support import SearchResult, all_edges_valid, reconstruct, validate_search


def weighted_search(graph, start, goal, heuristic, edge_is_valid):
    """Find the least-cost permitted graph route with a consistent heuristic.

    Inputs: graph[node] is an ordered tuple of (neighbor, cost) pairs.
    heuristic(node) returns a lower estimate of remaining cost.
    edge_is_valid(node, neighbor) says whether that proposed edge may be used.
    Supplied validation checks shape/range, not heuristic consistency.

    제공: 힙 삽입·제거, 반복문, 오래된 항목 건너뛰기, 간선 검사, 경로 복원·반환.
    node와 saved_g는 현재 노드와 시작부터의 비용이다.
    neighbor와 edge_cost는 검사할 이웃과 허용 간선의 비용이다.
    best는 발견한 최소 비용, parent는 경로의 이전 노드를 저장한다.

    TODO 1 — 비용 개선과 우선순위 작성
    - new_g에 이 간선을 거쳐 이웃에 도달하는 비용을 계산한다.
    - 기존 best가 없으면 무한대로 본다. 비용이 작아지지 않으면 continue로
      다음 이웃을 검사한다. 같은 비용이면 기존 parent를 유지한다.
    - 개선이면 이웃의 best와 parent를 갱신한다.
    - priority에 새 비용과 heuristic(neighbor)를 합한 우선순위를 저장한다.
    아래 제공된 heappush가 new_g와 priority를 사용한다. 입력과 이웃 순서는 보존한다.
    반환형: path는 노드 tuple 또는 None, expanded는 노드 tuple, cost는 float.
    expanded는 오래된 항목을 제외하고 실제 꺼낸 노드의 순서이며 goal 포함.
    """
    validate_search(graph, start, goal, heuristic, edge_is_valid)
    serial = count()
    frontier = [(float(heuristic(start)), next(serial), 0.0, start)]
    best = {start: 0.0}
    parent = {start: None}
    expanded = []
    while frontier:
        _, _, saved_g, node = heappop(frontier)
        if saved_g > best[node]:
            continue
        expanded.append(node)
        if node == goal:
            return SearchResult(reconstruct(parent, goal), saved_g,
                                tuple(expanded))
        for neighbor, edge_cost in graph[node]:
            if not edge_is_valid(node, neighbor):
                continue
            # ----- BEGIN TODO 1 -----
            raise NotImplementedError("TODO 1: replace with cost improvement and priority")
            # ----- END TODO 1 -----
            heappush(frontier, (priority, next(serial), new_g, neighbor))
    return SearchResult(None, float("inf"), tuple(expanded))


def dijkstra(graph, start, goal, edge_is_valid=all_edges_valid):
    return weighted_search(graph, start, goal, lambda node: 0.0, edge_is_valid)


class AStarPlanner:
    """Provided wrapper. Students edit weighted_search, not this class."""
    def __init__(self, heuristic, edge_is_valid=all_edges_valid):
        self.heuristic = heuristic
        self.edge_is_valid = edge_is_valid

    def plan(self, graph, start, goal):
        return weighted_search(graph, start, goal,
                               self.heuristic, self.edge_is_valid)

