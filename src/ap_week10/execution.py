"""Provided current-run recording. No student edits.

Physical execution and image-display status are separate. Every attempt has
a fresh camera-output directory, so old successful images cannot be reused.
"""
from datetime import datetime, timezone
from html import escape
import json
from pathlib import Path
from uuid import uuid4


def _utc():
    return datetime.now(timezone.utc).isoformat()


def _atomic_text(path, text):
    temporary = path.with_name(path.name + "." + uuid4().hex + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def _json(path, value):
    _atomic_text(path, json.dumps(value, indent=2, allow_nan=False) + "\n")


def _status_page(status, run_id, message=""):
    return ("<!doctype html><meta charset='utf-8'><title>Week 10 execution</title>"
            "<h1>Week 10 execution: " + escape(status) + "</h1>"
            "<p>Run: " + escape(run_id) + "</p><p>" + escape(message) + "</p>")


def _check_current_frames(current, report):
    frames = report.get("frames", [])
    if not frames:
        raise ValueError("No camera frames were recorded for this run")
    for frame in frames:
        image = (current / frame["file"]).resolve()
        if image.parent != current or not image.is_file():
            raise ValueError("A current-run camera frame is missing or invalid")


def execute_recorded(planner, output_dir, capture, path_provider, scene_runner,
                     preview_renderer=None, browser_opener=None):
    """Record planning/execution and handle display errors without denying a run."""
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_") + uuid4().hex[:12]
    current = output / "runs" / run_id
    current.mkdir(parents=True, exist_ok=False)
    metadata = {"run_id": run_id, "started_at_utc": _utc(),
                "planner": planner,
                "artifact_directory": current.relative_to(output).as_posix()}
    path_record = {**metadata, "status": "RUNNING"}
    _json(output / "result.json", {**metadata, "status": "RUNNING",
                                 "executed": False, "frames": []})
    _json(output / "path.json", path_record)
    _atomic_text(output / "preview.html",
                 _status_page("RUNNING", run_id, "No completed images for this run."))
    report = None
    try:
        points, planning = path_provider()
        candidate_path = {**path_record, "units": "rad",
                          "cost": float(planning.cost), "path": points.tolist()}
        json.dumps(candidate_path, allow_nan=False)
        path_record = candidate_path
        _json(output / "path.json", path_record)
        _json(current / "path.json", path_record)
        report = scene_runner(points, output_dir=current, capture=capture)
        if not isinstance(report, dict) or report.get("status") != "EXECUTED":
            raise RuntimeError("GPU runner did not return EXECUTED")
    except BaseException as error:
        failed = {**metadata, "status": "FAILED", "executed": False,
                  "finished_at_utc": _utc(), "frames": [],
                  "error": {"type": type(error).__name__, "message": str(error)}}
        if isinstance(report, dict):
            failed["runner_status"] = str(report.get("status"))
        path_record["status"] = "FAILED"
        _json(output / "result.json", failed)
        _json(current / "path.json", path_record)
        _json(output / "path.json", path_record)
        _json(current / "result.json", failed)
        _atomic_text(output / "preview.html",
                     _status_page("FAILED", run_id,
                                  type(error).__name__ + ": " + str(error)))
        raise

    # Physical execution has completed. Later capture/display errors must not
    # rewrite that fact as an execution failure.
    final_report = {**report, **metadata, "status": "EXECUTED", "executed": True,
                    "finished_at_utc": _utc(),
                    "capture_status": "PENDING" if capture else "NOT_REQUESTED",
                    "preview_status": "PENDING" if capture else "NOT_REQUESTED"}
    path_record["status"] = "EXECUTED"
    _json(current / "path.json", path_record)
    _json(output / "path.json", path_record)
    _json(current / "result.json", final_report)
    _json(output / "result.json", final_report)
    gallery = None
    preview = _status_page("EXECUTED", run_id,
                           "This run did not request camera capture.")
    if capture:
        try:
            _check_current_frames(current, report)
            final_report["capture_status"] = "SAVED"
            if preview_renderer is None:
                from .image_preview import show_saved_frames
                preview_renderer = show_saved_frames
            gallery = Path(preview_renderer(current, report, open_browser=False)).resolve()
            if gallery.parent != current or not gallery.is_file():
                raise RuntimeError("Current-run image gallery was not created")
            final_report["preview_status"] = "READY"
            relative_gallery = gallery.relative_to(output).as_posix()
            preview = (_status_page("EXECUTED", run_id) +
                       '<p><a href="' + escape(relative_gallery, quote=True) +
                       '">Open this run\'s saved frames</a></p>')
        except Exception as error:
            if final_report["capture_status"] == "PENDING":
                final_report["capture_status"] = "FAILED"
            final_report["preview_status"] = "FAILED"
            final_report["preview_error"] = {
                "type": type(error).__name__, "message": str(error)}
            gallery = None
            preview = _status_page(
                "EXECUTED", run_id,
                "Image preview unavailable: " + str(error) +
                ". Inspect the current-run directory: " + str(current))
            print("GPU execution completed, but image preview is unavailable:", str(error))
            print("Current-run files:", current)
    try:
        _atomic_text(output / "preview.html", preview)
    except Exception as error:
        final_report["preview_status"] = "FAILED"
        final_report["preview_error"] = {
            "type": type(error).__name__, "message": str(error)}
        print("GPU execution completed. Preview status page could not be written.")
        print("Current-run files:", current)
    _json(current / "result.json", final_report)
    _json(output / "result.json", final_report)

    if gallery is not None and browser_opener is not None:
        try:
            opened = browser_opener(gallery.as_uri())
            if opened is False:
                raise RuntimeError("The browser did not open the saved gallery")
            final_report["browser_status"] = "OPENED"
        except Exception as error:
            final_report["browser_status"] = "UNAVAILABLE"
            final_report["browser_error"] = str(error)
            print("Saved images are ready. Open this file manually:", gallery)
        _json(current / "result.json", final_report)
        _json(output / "result.json", final_report)
    return final_report

