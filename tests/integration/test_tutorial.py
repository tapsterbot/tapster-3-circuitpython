"""docs/tutorial.md: its `>>>` steps and its script run against the
simulated robot.

Keeps the tutorial honest: if a method it uses is renamed, or a command
starts failing, this fails.
"""

import re

import pytest

from fakes import ROOT

TUTORIAL = (ROOT / "docs" / "tutorial.md").read_text()
BLOCKS = re.findall(r"```python\n(.*?)```", TUTORIAL, re.S)


def repl_lines():
    """Every `>>> ` line, in order, except exit()."""
    lines = []
    for block in BLOCKS:
        for line in block.splitlines():
            if line.startswith(">>> ") and line != ">>> exit()":
                lines.append(line[4:])
    return lines


def script():
    (body,) = [b for b in BLOCKS if b.startswith("from tapster import Tapster")]
    return body


def test_tutorial_has_what_we_test():
    lines = repl_lines()
    for call in ("t.move(", "t.hop(", "t.home(", "t.tap(", "t.pbm.tap("):
        assert any(line.startswith(call) for line in lines), call


def test_repl_steps(robot):
    scope = {"t": robot.t}
    for line in repl_lines():
        result = eval(line, scope)
        if isinstance(result, dict):
            assert result.get("ok"), (line, result)
    assert robot.t.position == (0, 0, 0)  # last steps: home, tap, PBM
    assert robot.xyz() == pytest.approx((0, 0, 0), abs=0.3)


def test_square_script(robot):
    # the script makes its own Tapster(); the fixture's fake serial serves it
    mark = robot.mark()
    exec(compile(script(), "square.py", "exec"), {"__name__": "__main__"})
    corners = [p[:2] for p in robot.path(mark)]
    for x, y in [(-10, 10), (10, 10), (10, -10), (-10, -10)]:
        assert any(abs(cx - x) < 0.3 and abs(cy - y) < 0.3 for cx, cy in corners)
    assert robot.xyz() == pytest.approx((0, 0, 0), abs=0.3)  # home at the end
