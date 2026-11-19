import importlib.util
import re
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[4] / "tools"


def load_tool():
    spec = importlib.util.spec_from_file_location("tool_walk", TOOLS / "walk.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def number(out, label):
    return float(re.search(rf"{label}: (-?[0-9.]+)", out).group(1))


def test_a_default_walk_prints_the_metrics(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["walk", "--seconds", "6"])
    assert load_tool().main() == 0
    out = capsys.readouterr().out
    assert "gait: tripod" in out
    assert number(out, "forward") > 0.1
    assert number(out, "min stability margin") > 15.0
    assert number(out, "peak torque") < 1.47
    assert "max stance slip" in out and "heading change" in out


def test_gaits_and_directions_are_selectable(monkeypatch, capsys):
    monkeypatch.setattr(
        "sys.argv", ["walk", "--gait", "ripple", "--vx", "0", "--vy", "0.05", "--seconds", "8"]
    )
    assert load_tool().main() == 0
    out = capsys.readouterr().out
    assert "gait: ripple" in out
    assert number(out, "lateral") > 0.05


@pytest.mark.parametrize(
    "argv",
    [["--gait", "gallop"], ["--vx", "nan"], ["--omega", "inf"], ["--seconds", "-1"]],
)
def test_bad_input_prints_one_error_line(monkeypatch, capsys, argv):
    monkeypatch.setattr("sys.argv", ["walk", *argv])
    assert load_tool().main() == 1
    captured = capsys.readouterr()
    assert captured.err.startswith("walk: ") and len(captured.err.strip().splitlines()) == 1
