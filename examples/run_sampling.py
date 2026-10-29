"""Run TODOs 2/3 and compare the complete RRT-Connect under one budget."""
import argparse
from ap_week10.domain import PlanningProblem
from ap_week10.rrt import single_tree_rrt
from ap_week10.provided_planners import rrt_connect

problem = PlanningProblem()
parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, help="run one seed instead of 7, 11, 19")
args = parser.parse_args()
seeds = (7, 11, 19) if args.seed is None else (args.seed,)
for seed in seeds:
    print(f"seed={seed} | max_checks=200 | step_size=0.25 rad")
    for name, planner in (("RRT", single_tree_rrt), ("RRT-Connect", rrt_connect)):
        result = planner(problem, seed=seed, max_checks=200, step_size=0.25)
        print(f"{name}: {result.status} | cost={result.cost:.3f} rad"
              f" | checks={result.checks} | nodes={result.nodes}")
        print(f"  elapsed={result.elapsed_s:.6f} s (varies by PC/run)")

