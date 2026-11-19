import json
from dataclasses import replace

import numpy as np

from bemu_description.names import joint_order
from bemu_sim.torque_report import Scenario, ScenarioResult, render_markdown, to_json


def result(scenario, *, peak=0.5, rms=0.3, margin=0.08, joint=3):
    peaks = np.full(18, 0.1)
    peaks[joint] = peak
    rmss = np.full(18, 0.05)
    rmss[5] = rms
    return ScenarioResult(
        scenario=scenario,
        command=(0.12, 0.0, 0.0),
        joints=joint_order(),
        peak_torque=peaks,
        rms_torque=rmss,
        mean_abs_torque=np.full(18, 0.04),
        peak_speed=np.full(18, 1.0),
        steps_at_limit=np.zeros(18, dtype=int),
        min_margin=margin,
        max_slip=0.004,
        power=45.0,
    )


def results():
    good = result(Scenario("tripod", "forward", 1.0, 2.5))
    over = result(Scenario("ripple", "forward", 1.0, 2.5), peak=1.50)
    tipsy = result(Scenario("wave", "turn", 1.0, 2.5), margin=0.01)
    stand = replace(result(Scenario("tripod", "stand", 0.0, 2.5)), min_margin=None, max_slip=None)
    other_mass = result(Scenario("tripod", "forward", 1.0, 2.0))
    return [stand, good, over, tipsy, other_mass]


def test_render_markdown(params, actuators):
    text = render_markdown(results(), params, actuators, commit="abc1234", date="2026-11-18")
    for heading in ("# ", "## Method", "## Summary", "## 2.0 kg", "## 2.5 kg"):
        assert heading in text
    assert "abc1234" in text and "2026-11-18" in text
    assert "placeholder" in text.lower() and "supply_voltage_v" in text and "datasheet" in text
    assert "FAIL" in text and "UNSTABLE" in text and "PASS" in text
    assert joint_order()[3] in text
    assert "51%" in text  # 1.50 N·m over the 2.94 N·m stall
    for name in joint_order():
        assert name in text
    assert "5 rows" in text or "of 5" in text


def test_to_json_round_trips(params, actuators):
    data = to_json(results(), params, actuators, commit="abc1234", date="2026-11-18")
    assert json.loads(json.dumps(data)) == data
    assert data["commit"] == "abc1234"
    assert data["placeholders"] == ["supply_voltage_v"]
    assert len(data["results"]) == 5
    first = data["results"][2]
    assert set(first["peak_torque"]) == set(joint_order())
    assert set(first["rms_torque"]) == set(joint_order())
    assert first["verdict"]["peak_ok"] is False
    assert data["results"][3]["verdict"]["stable"] is False
    assert data["results"][0]["min_margin"] is None


def test_status_tags_land_on_the_right_rows(params, actuators):
    text = render_markdown(results(), params, actuators, commit="abc1234", date="2026-11-18")
    rows = {line.split("|")[1].strip(): line for line in text.splitlines() if line.startswith("| ")}
    assert rows["ripple forward 100%"].rstrip().endswith("FAIL |")
    assert rows["wave turn"].rstrip().endswith("PASS UNSTABLE |")
    assert rows["stand"].rstrip().endswith("PASS |")
    assert "3 of 5 rows pass" in text and "1 unstable" in text
