"""Provided two-joint planning problem and budget/result support."""
from dataclasses import dataclass, field
from time import perf_counter
import numpy as np
from .robot_constraints import slice_edge_is_allowed, slice_state_is_allowed


def point(value):
    result = np.array(value, dtype=np.float64, copy=True)
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError("a configuration must have two finite joint angles")
    return result


def distance(left, right):
    return float(np.linalg.norm(np.asarray(right) - np.asarray(left)))


@dataclass(frozen=True)
class PlanningProblem:
    start: np.ndarray = field(default_factory=lambda: np.array([-0.6, -0.8]))
    goal: np.ndarray = field(default_factory=lambda: np.array([0.6, -0.8]))
    bounds: np.ndarray = field(default_factory=lambda: np.array(
        [[-0.8, 0.8], [-1.2, -0.35]]))
    edge_is_valid: object = slice_edge_is_allowed
    state_is_valid: object = slice_state_is_allowed

    def __post_init__(self):
        start, goal = point(self.start), point(self.goal)
        bounds = np.array(self.bounds, dtype=float, copy=True)
        if bounds.shape != (2, 2) or not np.isfinite(bounds).all():
            raise ValueError("bounds must be finite shape (2, 2)")
        if np.any(bounds[:, 0] >= bounds[:, 1]):
            raise ValueError("each bound must have low < high")
        for q in (start, goal):
            if np.any(q < bounds[:, 0]) or np.any(q > bounds[:, 1]):
                raise ValueError("start and goal must be within bounds")
        if not callable(self.edge_is_valid):
            raise TypeError("edge_is_valid must be callable")
        if not callable(self.state_is_valid):
            raise TypeError("state_is_valid must be callable")
        if not all(self.state_is_valid(q) for q in (start, goal)):
            raise ValueError("start and goal must satisfy the state rule")
        for array in (start, goal, bounds):
            array.setflags(write=False)
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "goal", goal)
        object.__setattr__(self, "bounds", bounds)


@dataclass(frozen=True)
class PlanningResult:
    status: str
    path: np.ndarray | None
    cost: float
    checks: int
    nodes: int
    elapsed_s: float


class EdgeBudget:
    def __init__(self, callback, max_checks):
        if isinstance(max_checks, bool) or not isinstance(max_checks, int):
            raise ValueError("max_checks must be a nonnegative integer")
        if max_checks < 0:
            raise ValueError("max_checks must be nonnegative")
        self.callback = callback
        self.max_checks = max_checks
        self.used = 0

    @property
    def remaining(self):
        return self.max_checks - self.used

    def check(self, left, right):
        if not self.remaining:
            return False
        self.used += 1
        return bool(self.callback(left, right))


def prepare_tree(problem, seed, max_checks, step_size):
    if not np.isfinite(step_size) or step_size <= 0:
        raise ValueError("step_size must be finite and positive")
    rng = np.random.default_rng(seed)
    budget = EdgeBudget(problem.edge_is_valid, max_checks)
    return rng, budget, [problem.start.copy()], [None], perf_counter()


def tree_path(nodes, parents, index):
    indices = []
    while index is not None:
        indices.append(index)
        index = parents[index]
    return np.array([nodes[i] for i in reversed(indices)], dtype=np.float64)


def finish(path, budget, node_count, started):
    if path is None:
        return PlanningResult("BUDGET_EXHAUSTED", None, float("inf"),
                              budget.used, node_count, perf_counter() - started)
    path = np.array(path, dtype=np.float64, copy=True)
    cost = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    return PlanningResult("FOUND", path, cost, budget.used,
                          node_count, perf_counter() - started)

