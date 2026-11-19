import sys

from bemu_teleop import keyboard


class NotATerminal:
    def isatty(self):
        return False


def test_a_non_terminal_stdin_is_refused_in_one_line(monkeypatch, capsys):
    monkeypatch.setattr(sys, "stdin", NotATerminal())
    assert keyboard.main() == 1
    err = capsys.readouterr().err.strip().splitlines()
    assert len(err) == 1 and err[0].startswith("keyboard: ")


def test_the_help_lists_every_key():
    for key in ("w/s", "a/d", "q/e", "space", "1/2/3", "r/f", "h", "j", "x"):
        assert key in keyboard.HELP
