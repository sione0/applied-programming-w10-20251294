"""Display only actual frames saved by the supplied camera renderer."""

from html import escape
from pathlib import Path
import webbrowser


def show_saved_frames(output_dir, result, open_browser=True):
    """Write a local HTML gallery, then optionally open the saved page."""
    out = Path(output_dir).resolve()
    cards = []
    for frame in result.get('frames', []):
        name = frame['file']
        candidate = (out / name).resolve()
        if candidate.parent != out or not candidate.is_file():
            raise ValueError('A rendered frame is missing or outside the output directory')
        cards.append(f'<figure><img src="{escape(name, quote=True)}" alt="Saved Panda frame">'
                     f'<figcaption>Waypoint {frame["waypoint"]}, simulation step {frame["step"]}'
                     '</figcaption></figure>')
    if not cards:
        raise ValueError('No rendered frames were saved. Use the capture option.')
    page = out / 'preview.html'
    page.write_text('<!doctype html><html lang="en"><meta charset="utf-8">'
        '<title>Week 10 — saved GPU frames</title><style>'
        'body{font:18px Georgia,serif;margin:2rem;color:#111;background:white}'
        'figure{margin:1rem 0;padding:1rem;border:1px solid #222}'
        'img{max-width:100%;height:auto}figcaption{margin-top:.5rem}'
        '</style><h1>Week 10 — saved GPU frames</h1>'
        '<p>Actual camera captures from the supplied Genesis run. '
        'This page shows still images, not live robot control.</p>'
        + ''.join(cards) + '</html>', encoding='utf-8')
    if open_browser:
        webbrowser.open(page.as_uri())
    return page

