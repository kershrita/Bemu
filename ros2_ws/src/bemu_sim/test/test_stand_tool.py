import importlib.util
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[4] / "tools"


def load_tool():
    spec = importlib.util.spec_from_file_location("tool_stand", TOOLS / "stand.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stand_prints_a_report(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["stand", "stand"])
    assert load_tool().main() == 0
    out = capsys.readouterr().out
    assert "height: 98." in out
    assert "planted feet: 6" in out


def test_pose_with_an_impossible_roll_prints_one_error_line(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["stand", "pose", "--roll", "95"])
    assert load_tool().main() == 1
    captured = capsys.readouterr()
    assert captured.err.startswith("stand: ") and len(captured.err.strip().splitlines()) == 1


def test_sit_reports_the_resting_height(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["stand", "sit"])
    assert load_tool().main() == 0
    assert "height: 28." in capsys.readouterr().out
