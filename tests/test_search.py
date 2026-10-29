"""Supplied grouped checks. Students do not write tests this week."""
from copy import deepcopy
import math
from ap_week10.cases import GRAPH, H
from ap_week10.search import dijkstra, AStarPlanner
from ap_week10.support import all_edges_valid


def test_least_cost_and_astar_agreement():
    uniform = dijkstra(GRAPH, "S", "G")
    informed = AStarPlanner(H.__getitem__).plan(GRAPH, "S", "G")
    for result in (uniform, informed):
        assert result.path == ("S", "B", "A", "G")
        assert math.isclose(result.cost, 5.0, rel_tol=1e-12, abs_tol=1e-12)
    # The Reading's ordered graph: g and g+h reach the same path differently.
    assert uniform.expanded == ("S", "B", "C", "A", "D", "G"), (
        "Dijkstra must remove current-best entries in g priority order")
    assert informed.expanded == ("S", "B", "A", "G"), (
        "A* must use new_g + heuristic(neighbor), not new_g alone")


def test_improvement_stale_entries_and_tie_order():
    graph = {"S": (("A", 8), ("B", 1), ("C", 1)),
             "A": (("G", 10),), "B": (("A", 1),),
             "C": (("A", 1),), "G": ()}
    result = dijkstra(graph, "S", "G")
    assert result.path == ("S", "B", "A", "G")
    assert result.cost == 12
    assert result.expanded.count("A") == 1


def test_edge_validation_and_no_route():
    result = dijkstra(GRAPH, "S", "G", lambda left, right: right != "A")
    assert result.path == ("S", "B", "G")
    assert result.cost == 9
    missing = dijkstra(GRAPH, "S", "X", all_edges_valid)
    assert missing.path is None and math.isinf(missing.cost)


def test_same_start_goal_and_input_preservation():
    before = deepcopy(GRAPH)
    result = dijkstra(GRAPH, "S", "S")
    assert result.path == ("S",) and result.cost == 0
    assert GRAPH == before

