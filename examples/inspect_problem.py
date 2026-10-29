"""Complete input inspection. Does not call unfinished algorithms."""
from ap_week10.cases import GRAPH
from ap_week10.domain import PlanningProblem
from ap_week10.robot_constraints import slice_edge_is_allowed

problem = PlanningProblem()
print("Graph nodes:", ", ".join(GRAPH))
print("S neighbors:", GRAPH["S"])
print("Joint bounds (rad):", problem.bounds.tolist())
print("Start -> goal:", problem.start.tolist(), "->", problem.goal.tolist())
print("Direct edge allowed:", slice_edge_is_allowed(problem.start, problem.goal))

