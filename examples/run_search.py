"""Run TODO 1 on the teaching graph and the supplied robot roadmap."""
from ap_week10.cases import GRAPH, H
from ap_week10.search import dijkstra, AStarPlanner
from ap_week10.application import roadmap_result

for name, result in (("Dijkstra", dijkstra(GRAPH, "S", "G")),
                     ("A*", AStarPlanner(H.__getitem__).plan(GRAPH, "S", "G"))):
    print(f"{name}: {' -> '.join(result.path)} | cost={result.cost:.1f}")
    print("  expanded:", " ".join(result.expanded))
result = roadmap_result()
print(f"Robot roadmap: {' -> '.join(result.path)} | cost={result.cost:.3f} rad")

