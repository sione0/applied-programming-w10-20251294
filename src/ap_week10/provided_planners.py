"""Complete comparison planner. No student edits."""
import numpy as np
from .domain import distance, prepare_tree, tree_path, finish


def _provided_steer(origin, target, step):
    delta = target - origin
    length = float(np.linalg.norm(delta))
    if length <= step:
        return np.array(target, copy=True)
    return origin + delta * step / length


def rrt_connect(problem, seed=7, max_checks=200, step_size=0.25):
    """Alternate two trees, greedily extending the other toward each new node.

    Every attempted edge consumes the SAME supplied edge-check budget used by
    single_tree_rrt. Internal extension count is not treated as free computation.
    Like basic RRT, this comparison is not an optimal planner.
    """
    rng, budget, root, parents, started = prepare_tree(
        problem, seed, max_checks, step_size)
    if np.array_equal(problem.start, problem.goal):
        return finish(np.array(root), budget, 1, started)
    trees = [(root, parents), ([problem.goal.copy()], [None])]

    def extend(tree, target):
        nodes, links = tree
        nearest = min(range(len(nodes)),
                      key=lambda i: distance(nodes[i], target))
        if np.array_equal(nodes[nearest], target):
            return nearest, True
        new_state = _provided_steer(nodes[nearest], target, step_size)
        if not budget.check(nodes[nearest], new_state):
            return None, False
        nodes.append(new_state)
        links.append(nearest)
        return len(nodes) - 1, np.array_equal(new_state, target)

    active = 0
    while budget.remaining:
        target = rng.uniform(problem.bounds[:, 0], problem.bounds[:, 1])
        first_index, _ = extend(trees[active], target)
        if first_index is not None:
            target = trees[active][0][first_index]
            while budget.remaining:
                other_index, reached = extend(trees[1 - active], target)
                if other_index is None:
                    break
                if reached:
                    first = tree_path(*trees[active], first_index)
                    other = tree_path(*trees[1 - active], other_index)
                    if active == 0:
                        path = np.vstack((first, other[-2::-1]))
                    else:
                        path = np.vstack((other, first[-2::-1]))
                    return finish(path, budget,
                                  sum(len(t[0]) for t in trees), started)
        active = 1 - active
    return finish(None, budget, sum(len(t[0]) for t in trees), started)

