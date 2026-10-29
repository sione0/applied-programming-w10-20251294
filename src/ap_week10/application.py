"""Complete graph-to-configuration adapter and output helpers."""
import numpy as np
from .cases import make_roadmap, ROADMAP_POSITIONS
from .domain import PlanningProblem
from .robot_constraints import slice_edge_is_allowed, validate_robot_path
from .search import AStarPlanner
from .rrt import single_tree_rrt


def roadmap_result():
    graph = make_roadmap()
    goal = ROADMAP_POSITIONS["G"]
    planner = AStarPlanner(
        lambda node: float(np.linalg.norm(ROADMAP_POSITIONS[node] - goal)),
        lambda left, right: slice_edge_is_allowed(
            ROADMAP_POSITIONS[left], ROADMAP_POSITIONS[right]))
    return planner.plan(graph, "S", "G")


def selected_path(planner):
    if planner == "astar":
        result = roadmap_result()
        if result.path is None:
            raise RuntimeError("A* did not find a roadmap path")
        points = np.array([ROADMAP_POSITIONS[name] for name in result.path])
    elif planner == "rrt":
        result = single_tree_rrt(PlanningProblem(), seed=7, max_checks=200)
        if result.path is None:
            raise RuntimeError("RRT exhausted the edge-check budget")
        points = result.path
    else:
        raise ValueError("planner must be astar or rrt")
    return validate_robot_path(points), result

