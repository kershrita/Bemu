"""Write the generated model files to disk."""

from __future__ import annotations

from pathlib import Path

from bemu_description.actuators import ActuatorModel
from bemu_description.mjcf import build_leg_mjcf
from bemu_description.urdf import build_leg_urdf
from bemu_params import RobotParams


def write_models(
    params: RobotParams, actuators: ActuatorModel, out_dir: Path, *, headless: bool = False
) -> dict[str, Path]:
    """Write leg.xml (MJCF) and leg.urdf into out_dir; the URDF names the MJCF by absolute path."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    mjcf_path = (out_dir / "leg.xml").resolve()
    urdf_path = (out_dir / "leg.urdf").resolve()
    mjcf_path.write_text(build_leg_mjcf(params, actuators), encoding="utf-8")
    urdf_path.write_text(
        build_leg_urdf(params, mujoco_model_path=str(mjcf_path), headless=headless),
        encoding="utf-8",
    )
    return {"mjcf": mjcf_path, "urdf": urdf_path}
