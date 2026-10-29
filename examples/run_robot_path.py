"""Execute one completed planner path with supplied current-run recording."""
import argparse
from pathlib import Path
import sys
import webbrowser
from ap_week10.execution import execute_recorded


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner", choices=("astar", "rrt"), default="astar")
    parser.add_argument("--capture", action="store_true")
    args = parser.parse_args()
    output = Path("artifacts") / args.planner

    def plan():
        from ap_week10.application import selected_path
        return selected_path(args.planner)

    def render(points, output_dir, capture):
        from ap_week10.robot_scene import run_scene
        return run_scene(points, output_dir=output_dir, capture=capture)

    try:
        report = execute_recorded(args.planner, output, args.capture, plan, render,
                                  browser_opener=webbrowser.open)
    except Exception as error:
        print("GPU execution FAILED:", str(error), file=sys.stderr)
        print("Run report:", output / "result.json", file=sys.stderr)
        return 1
    print("GPU execution complete")
    print("Path file:", output / "path.json")
    print("Execution status:", report["status"])
    print("Run report:", output / "result.json")
    if args.capture:
        print("Image preview:", output / "preview.html")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

