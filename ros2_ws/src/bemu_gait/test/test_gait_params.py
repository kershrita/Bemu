import pytest
import yaml

from bemu_gait import GaitParamsError, load_gait_params
from bemu_gait.params import DEFAULT_PATH


def edited(tmp_path, **changes):
    data = yaml.safe_load(DEFAULT_PATH.read_text())
    for dotted, value in changes.items():
        *parents, last = dotted.split(".")
        node = data
        for part in parents:
            node = node[part]
        if value is None:
            del node[last]
        else:
            node[last] = value
    out = tmp_path / "gait.yaml"
    out.write_text(yaml.safe_dump(data))
    return out


def test_loads_in_si(gait_params):
    assert gait_params.cycle_time == {"tripod": 1.0, "ripple": 0.75, "wave": 0.6}
    assert gait_params.step_height == pytest.approx(0.030)
    assert gait_params.max_stride == pytest.approx(0.060)
    assert gait_params.max_linear_accel == pytest.approx(0.2)
    assert gait_params.max_angular_accel == pytest.approx(1.0)
    assert gait_params.lift_ramp_stride == pytest.approx(0.010)
    assert gait_params.blend_cycles == pytest.approx(1.0)
    assert gait_params.stability_threshold == pytest.approx(0.015)


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("step_height_mm", None),
        ("step_height_mm", -1),
        ("step_height_mm", "tall"),
        ("max_stride_mm", 0),
        ("blend_cycles", float("nan")),
        ("cycle_time_s.wave", None),
        ("cycle_time_s.wave", 0),
        ("cycle_time_s.gallop", 1.0),
        ("cycle_time_s", None),
    ],
)
def test_bad_files_raise_naming_the_key(tmp_path, key, value):
    path = edited(tmp_path, **{key: value})
    with pytest.raises(GaitParamsError, match=key.split(".")[0]):
        load_gait_params(path)


def test_missing_file_is_a_params_error(tmp_path):
    with pytest.raises(GaitParamsError, match="nope.yaml"):
        load_gait_params(tmp_path / "nope.yaml")
