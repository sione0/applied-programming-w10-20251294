"""TODOs 2 and 3: bounded steering and a single growing tree."""
import numpy as np
from .domain import point, distance, prepare_tree, tree_path, finish


def steer(q_from, q_target, step_size):
    """Return a new (2,) float64 point at most step_size toward q_target.

    TODO 2 — 작성할 내용
    - target까지 거리가 step_size 이하이면 target의 독립 복사 반환.
    - 더 멀면 방향을 정규화하여 step_size만큼 이동한 새 배열 반환.
    - 같은 점일 때 0으로 나누지 않음. 두 입력은 변경하지 않음.
    사용할 것: np.linalg.norm, 배열의 + / *, copy().
    """
    origin, target = point(q_from), point(q_target)
    if not np.isfinite(step_size) or step_size <= 0:
        raise ValueError("step_size must be finite and positive")
    # ----- BEGIN TODO 2 -----
    raise NotImplementedError("TODO 2: replace with your steering calculation")
    # ----- END TODO 2 -----


def single_tree_rrt(problem, seed=7, max_checks=200, step_size=0.25):
    """Build one tree from start, using the supplied edge-check budget.

    problem.start/goal: (2,) joint angles. problem.bounds: low/high for each joint.
    The provided constructor validates start and goal before planning.
    nodes contains states. parents[i] is the index that first connected nodes[i].
    budget.check(a,b) counts one proposed edge and calls the provided validator.
    budget.remaining is the number of checks still available.

    TODO 3 — 작성할 내용
    - rng.random() < 0.15이면 goal, 아니면 rng.uniform(low, high)로 목표 표본 선택.
      low = problem.bounds[:, 0], high = problem.bounds[:, 1]로 각 관절 범위 사용.
    - nodes에서 표본과 거리가 가장 가까운 상태 선택. 같으면 먼저 추가된 상태.
    - steer 호출 후 그 간선을 budget.check로 검사. 거부되면 다음 표본으로 진행.
    - 허용된 새 상태와 부모 인덱스를 nodes/parents에 각각 추가.
    - 새 상태가 goal이면 경로 반환.
    - goal까지 거리가 step_size 이하이고 검사 여유가 있으면 마지막 간선 검사.
      허용될 때 goal과 부모를 추가하고 경로 반환.
    - 검사 예산이 끝나면 제공된 실패 반환문으로 종료.
    사용할 것: distance, steer, tree_path(nodes, parents, last_index),
               finish(path, budget, len(nodes), started).
    rng/초기 트리/실패 반환은 제공됨. 입력 배열과 problem은 변경하지 않음.
    """
    rng, budget, nodes, parents, started = prepare_tree(
        problem, seed, max_checks, step_size)
    if np.array_equal(problem.start, problem.goal):
        return finish(np.array(nodes), budget, len(nodes), started)
    while budget.remaining:
        # ----- BEGIN TODO 3 -----
        raise NotImplementedError("TODO 3: replace with your tree expansion")
        # ----- END TODO 3 -----
    return finish(None, budget, len(nodes), started)

