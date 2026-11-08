"""Smoke test for tools/pi_drive.py in --sim mode (synthetic scans, FakeMotor, loopback)."""
from __future__ import annotations

import importlib.util
from pathlib import Path

from locatebot.pi.motor import FakeMotor

_PATH = Path(__file__).resolve().parents[2] / "tools" / "pi_drive.py"
_spec = importlib.util.spec_from_file_location("pi_drive", _PATH)
pi_drive = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pi_drive)


def test_sim_run_exits_with_motor_stopped(capsys):
    m = FakeMotor()
    assert pi_drive.main(["--sim", "--port", "0", "--seconds", "0.3", "--quiet"], motor=m) == 0
    assert m.closed and m.stop_count >= 1 and m.last == (0.0, 0.0)
    assert len(m.outputs) >= 3  # the loop ticked
    assert all(o == (0.0, 0.0) for o in m.outputs)  # nobody drove: always stopped
    assert "127.0.0.1" in capsys.readouterr().out
