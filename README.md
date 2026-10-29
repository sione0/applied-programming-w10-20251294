# Week 10: Priority Queues and Cost-Aware Search

Read the [Reading material and integrated Assignment](docs/week10_reading_v23.pdf).
The complete runnable queue example is supplied, with no TODOs to complete first.

## 1. Purpose and files to edit

Implement cost improvement and priority selection inside the supplied Dijkstra/A*
search loop to find a least-cost permitted graph route. Then grow a
single RRT from the start configuration and compare it with the supplied
RRT-Connect. Execute one completed path in the existing GPU environment.

Edit only these marked regions:

1. `src/ap_week10/search.py` — TODO 1: cost improvement and priority in `weighted_search`.
2. `src/ap_week10/rrt.py` — TODO 2 in `steer` and TODO 3 in `single_tree_rrt`.

Remove each corresponding `raise NotImplementedError(...)` and write your code
between its BEGIN/END comments. Keep the signatures and supplied code.
The Reading explains the algorithms. The comments above each function state
the exact inputs, required actions and returned values.

The search loop and heap operations, validation, parent-chain reconstruction,
result objects, comparison planner, robot execution and tests are supplied.
Do not edit them or add student tests, a wheel, an engineering note or a separate report.

## 2. Prepare the repository and existing environment

Use **Use this template** on this starter to
create your private `applied-programming-w10-YOUR-STUDENT-ID` repository.
Invite the exact collaborator account `SSUMechE`.

Open **Anaconda Prompt**. Replace the course folder, GitHub ID and student ID
with your own values. Run each command on its own line.

```bat
cd /d "C:\your-course-folder"
git clone https://github.com/YOUR-GITHUB-ID/applied-programming-w10-YOUR-STUDENT-ID.git
cd applied-programming-w10-YOUR-STUDENT-ID
git remote -v
```

Both origin addresses must identify your own private Week 10 repository.
Use the same **applied-programming-cuda** environment created by the earlier
all-in-one CUDA/Genesis command for every example, test and robot run.
Do not create a Week 10 environment or change the installed PyTorch, CUDA,
Genesis or NumPy.

From this repository root:

```bat
set PYTHONUTF8=1
conda activate applied-programming-cuda
python -m pip install "setuptools>=80.9.0" pytest==9.1.1
python -m pip install -e . --no-deps --no-build-isolation
python scripts/verify_environment.py
```

Expected status line: `ENVIRONMENT PASS`. This checks the existing environment,
the checkout location and a small CUDA operation. It does not call unfinished
TODOs or establish algorithm correctness. The two installation commands ensure
the test/build tools are present and register this checkout without replacing
the shared GPU dependencies.

If the environment is missing, use the previously distributed
[all-in-one setup command](https://github.com/SSUMechE/applied-programming-2026-week05-starter/releases/download/cuda-setup-v12-20261012/INSTALL_COMMAND.txt)
in Anaconda Prompt, then return here. Do not rerun that installer for an existing
environment. Report an import/version failure instead of independently replacing
PyTorch or changing security settings.

## 3. Inspect the supplied examples

These complete examples run before any TODO is finished:

```bat
python examples/heap_demo.py
python examples/inspect_problem.py
```

The heap example prints:

```text
FIFO first: A
1 B
2 C
4 A
```

The problem inspection shows the ordered graph, the two joint-angle bounds and
`Direct edge allowed: False`. The graph weights are teaching cost units.
The robot example uses joint angles in radians, not Cartesian tool positions.

## 4. Complete and check the algorithms

### TODO 1: cost improvement and priority

Edit only the marked region inside the neighbor loop in `weighted_search`.
Compute `new_g`, then use `continue` to skip a neighbor whose cost does not
strictly improve. Otherwise update its `best` and `parent` and assign `priority`
from the new cost plus `heuristic(neighbor)`. The supplied heap insertion uses
`new_g` and `priority`. The loop, stale-entry handling and returns are supplied.
Dijkstra supplies a zero heuristic. `AStarPlanner` supplies the given heuristic
to the same function.

```bat
python examples/run_search.py
python -m pytest -q tests/test_search.py
```

Expected program output:

```text
Dijkstra: S -> B -> A -> G | cost=5.0
  expanded: S B C A D G
A*: S -> B -> A -> G | cost=5.0
  expanded: S B A G
Robot roadmap: S -> L -> M -> R -> G | cost=1.321 rad
```

The grouped check ends with `4 passed`. The roadmap uses the same completed
search and the supplied whole-edge validator.

### TODOs 2 and 3: steering and a single tree

Complete `steer`, then the marked tree-growth body in `single_tree_rrt`.
The root, random generator, budget and final failure result are supplied.

```bat
python examples/run_sampling.py
python -m pytest -q tests/test_rrt.py
```

The command compares both planners for seeds 7, 11 and 19. Selected result lines
for seed 7, step size 0.25 rad and 200 edge checks:

```text
RRT: FOUND | cost=1.504 rad | checks=28 | nodes=22
RRT-Connect: FOUND | cost=1.585 rad | checks=18 | nodes=11
```

Use `python examples/run_sampling.py --seed 7` to inspect only this case.
The program also prints its inputs and elapsed seconds. Elapsed time varies
by PC and run. It is not a required speed ratio or a grading threshold.
Valid implementations may grow different trees. Path validity and budget behavior,
not an exact path, node count or elapsed time, determine correctness.
The grouped check ends with `4 passed`.

Keep the same problem, seed setting and edge-check budget when comparing the
planners. RRT-Connect is supplied, not a fourth TODO. Neither sampling planner
is required to produce an optimal path. A budget-exhausted run does not prove
that no valid route exists.

## 5. Run a completed path on the GPU

This execution is required. Use the completed A* path below, or replace
`astar` with `rrt` after its TODOs are complete.

```bat
python examples/run_robot_path.py --planner astar --capture
```

The supplied runner moves the Panda robot through the computed joint states,
saves actual camera images and opens their preview. It does not display an
invented animation. Check the reported execution status and inspect the saved
frames. The software joint-region rule is not a physical obstacle model or a
real-robot safety guarantee.

Generated files are under `artifacts/astar` (or `artifacts/rrt`):

- `path.json`: the planned joint path and cost.
- `result.json`: the observed execution and supplied checks.
- `preview.html`: current-run status and a link to its saved camera frames.
- `runs/<run-id>/`: this run's `trace.csv`, waypoint PNG files and image gallery.

Each new attempt replaces the current status with RUNNING, then EXECUTED or
FAILED. Earlier run folders are preserved, but a failed attempt does not display
an earlier successful run as its result.
If physical execution completes but image capture or display fails, the report
retains EXECUTED and records the separate capture/preview error. Inspect the
current-run folder named in the message.

These files support checking your work. They are not separate LMS uploads.

## 6. Final check and submission

Run the provided final check:

```bat
python -m pytest -q
```

Expected summary: `8 passed`. Passing the algorithm checks does not replace
the required GPU execution or claim real-robot safety.

From your private repository root:

```bat
git diff
git status --short
git add src/ap_week10/search.py src/ap_week10/rrt.py
git diff --staged
git commit -m "Complete Week 10 search algorithms"
git push origin main
git status --short
git rev-parse HEAD
git ls-remote origin refs/heads/main
```

Confirm that the two full commit IDs match and that your private repository is
accessible to `SSUMechE`. An accurate invitation sent on time may still be
awaiting instructor acceptance and is not a student omission.

Submit these two items in LMS:

- Your private repository URL.
- The matching full 40-character commit ID.

The deadline is one week after the laboratory session. Follow the exact date
and time in LMS. GitHub upload alone is not an LMS submission. Required naming,
file and LMS requirements must be met by the deadline, otherwise the assignment
receives zero under the announced policy. No additional ZIP, screenshots or
report is required.

