import importlib.util
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[4] / "tools"


def load_tool():
    spec = importlib.util.spec_from_file_location("tool_torque_report", TOOLS / "torque_report.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tool_quick_run_writes_both_files(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr("sys.argv", ["torque_report", "--quick", "--out", str(tmp_path)])
    assert load_tool().main() == 0
    markdown = (tmp_path / "v1-torque-report.md").read_text()
    assert (tmp_path / "v1-torque-report.json").exists()
    assert "tripod" in markdown and "stand" in markdown and "wave" in markdown
    assert "turn" in markdown and "forward" in markdown


def test_tool_reports_an_unwritable_output_dir_in_one_line(monkeypatch, tmp_path, capsys):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    monkeypatch.setattr("sys.argv", ["torque_report", "--quick", "--out", str(blocker / "sub")])
    assert load_tool().main() == 1
    err = capsys.readouterr().err.strip().splitlines()
    assert len(err) == 1 and err[0].startswith("torque_report: ")
