"""Supplied state/edge/budget checks, not exact timed-performance targets."""
import numpy as np
from types import SimpleNamespace
from ap_week10.domain import PlanningProblem
from ap_week10 import rrt as rrt_module
from ap_week10.rrt import steer, single_tree_rrt


def assert_valid_result(result, problem):
    assert result.status == "FOUND"
    np.testing.assert_allclose(result.path[0], problem.start, rtol=0, atol=1e-12)
    np.testing.assert_allclose(result.path[-1], problem.goal, rtol=0, atol=1e-12)
    assert all(problem.edge_is_valid(a, b)
               for a, b in zip(result.path[:-1], result.path[1:]))
    expected = np.linalg.norm(np.diff(result.path, axis=0), axis=1).sum()
    np.testing.assert_allclose(result.cost, expected, rtol=1e-12, atol=1e-12)


def test_steer_distance_dtype_and_independent_results():
    origin, target = np.array([0., 0.]), np.array([3., 4.])
    actual = steer(origin, target, 1.0)
    np.testing.assert_allclose(actual, [.6, .8], rtol=1e-12, atol=1e-12)
    assert actual.dtype == np.float64 and actual.shape == (2,)
    for a, b in ((origin, target), (target, target)):
        result = steer(a, b, 10.0)
        np.testing.assert_allclose(result, b, rtol=0, atol=0)
        result[0] = 99
        assert not np.shares_memory(result, a)
        assert not np.shares_memory(result, b)
    np.testing.assert_array_equal(origin, [0, 0])
    np.testing.assert_array_equal(target, [3, 4])


def _assert_sampling_branch(monkeypatch, random_value, expected_endpoint):
    """One rejected attempt checks the supplied goal/uniform rule, not a tree."""
    proposals = []

    def blocked(left, right):
        proposals.append((left.copy(), right.copy()))
        return False

    problem = PlanningProblem(edge_is_valid=blocked)

    def uniform(low, high, size=None):
        np.testing.assert_array_equal(low, problem.bounds[:, 0])
        np.testing.assert_array_equal(high, problem.bounds[:, 1])
        sample = np.array([-.6, -1.2])
        return sample if size is None else np.broadcast_to(sample, size).copy()

    samples = SimpleNamespace(random=lambda: random_value, uniform=uniform)
    original_prepare = rrt_module.prepare_tree

    def prepare(*args):
        _, budget, nodes, parents, started = original_prepare(*args)
        return samples, budget, nodes, parents, started

    with monkeypatch.context() as patcher:
        patcher.setattr(rrt_module, "prepare_tree", prepare)
        result = single_tree_rrt(problem, max_checks=1, step_size=.25)

    assert result.checks == len(proposals) == 1
    assert result.status == "BUDGET_EXHAUSTED" and result.path is None
    assert result.nodes == 1, "A rejected edge must not add a tree node"
    np.testing.assert_allclose(proposals[0][0], problem.start, rtol=0, atol=1e-12)
    np.testing.assert_allclose(proposals[0][1], expected_endpoint,
                               rtol=0, atol=1e-12,
                               err_msg="Apply the < 0.15 goal rule, then a bounded step")


def test_tree_sampling_policy_steps_and_valid_path(monkeypatch):
    base = PlanningProblem()
    proposals = []

    def checked(left, right):
        proposals.append(float(np.linalg.norm(right - left)))
        return base.edge_is_valid(left, right)

    problem = PlanningProblem(edge_is_valid=checked)
    step_size = .25
    result = single_tree_rrt(problem, seed=7, max_checks=200, step_size=step_size)
    assert all(length <= step_size + 1e-12 for length in proposals), (
        "A proposed edge must fit step_size even when validity rejects it")
    assert_valid_result(result, problem)
    assert 0 < result.checks <= 200
    lengths = np.linalg.norm(np.diff(result.path, axis=0), axis=1)
    assert np.all(lengths <= step_size + 1e-12), (
        "Every tree edge, including the final goal connection, must fit step_size")
    _assert_sampling_branch(monkeypatch, np.nextafter(.15, 0), [-.35, -.8])
    _assert_sampling_branch(monkeypatch, .15, [-.6, -1.05])


def test_determinism_and_input_preservation():
    problem = PlanningProblem()
    before = (problem.start.copy(), problem.goal.copy(), problem.bounds.copy())
    a = single_tree_rrt(problem, seed=7)
    b = single_tree_rrt(problem, seed=7)
    np.testing.assert_array_equal(a.path, b.path)
    assert (a.checks, a.nodes, a.cost) == (b.checks, b.nodes, b.cost)
    for actual, expected in zip((problem.start, problem.goal, problem.bounds), before):
        np.testing.assert_array_equal(actual, expected)


def test_budget_failure_and_same_start_goal():
    calls = []
    def blocked(left, right):
        calls.append((left.copy(), right.copy()))
        return False
    problem = PlanningProblem(edge_is_valid=blocked)
    result = single_tree_rrt(problem, max_checks=5)
    assert result.status == "BUDGET_EXHAUSTED" and result.path is None
    assert result.checks == len(calls) == 5
    result = single_tree_rrt(problem, max_checks=0)
    assert result.checks == 0 and result.path is None
    same = PlanningProblem(goal=(-.6, -.8))
    result = single_tree_rrt(same, max_checks=0)
    assert result.status == "FOUND" and result.cost == 0
    assert result.path.shape == (1, 2)

