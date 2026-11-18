import importlib.util
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[4] / "tools"


def load_tool(name):
    spec = importlib.util.spec_from_file_location(f"tool_{name}", TOOLS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_generate_model_reports_unwritable_output_in_one_line(tmp_path, monkeypatch, capsys):
    blocker = tmp_path / "file"
    blocker.write_text("not a directory")
    monkeypatch.setattr("sys.argv", ["generate_model", "--out", str(blocker / "model")])
    with pytest.raises(SystemExit) as info:
        load_tool("generate_model").main()
    assert info.value.code == 1
    err = capsys.readouterr().err
    assert err.startswith("generate_model: ") and len(err.strip().splitlines()) == 1


def test_trace_foot_reports_bad_parameter_file_in_one_line(tmp_path, monkeypatch, capsys):
    tool = load_tool("trace_foot")
    monkeypatch.setattr(tool, "PARAMS_FILE", tmp_path / "missing.yaml")
    monkeypatch.setattr("sys.argv", ["trace_foot", "--shape", "square"])
    assert tool.main() == 1
    err = capsys.readouterr().err
    assert err.startswith("trace_foot: ") and len(err.strip().splitlines()) == 1
