"""Write the generated model files to disk."""

from __future__ import annotations

from pathlib import Path

from bemu_description.actuators import ActuatorModel
from bemu_description.mjcf import build_leg_mjcf
from bemu_description.robot_mjcf import build_robot_mjcf
from bemu_description.urdf import build_leg_urdf, build_robot_urdf
from bemu_description.visuals import load_visuals
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


def write_robot_mjcf(
    params: RobotParams, actuators: ActuatorModel, out_dir: Path, *, visuals=None
) -> Path:
    """Write robot.xml (the full 18-joint MuJoCo model) into out_dir and return its path."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = (out_dir / "robot.xml").resolve()
    path.write_text(build_robot_mjcf(params, actuators, visuals=visuals), encoding="utf-8")
    return path


def write_robot_models(
    params: RobotParams,
    actuators: ActuatorModel,
    out_dir: Path,
    *,
    headless: bool = False,
    show_cad: bool | None = None,
) -> dict[str, Path]:
    """Write robot.xml (MJCF) and robot.urdf into out_dir; the URDF names the MJCF by full path.
    With `show_cad` (default: when a window is shown) the CAD meshes are drawn in both."""
    show = (not headless) if show_cad is None else show_cad
    visuals = load_visuals() if show else None
    mjcf_path = write_robot_mjcf(params, actuators, out_dir, visuals=visuals)
    urdf_path = (Path(out_dir) / "robot.urdf").resolve()
    urdf_path.write_text(
        build_robot_urdf(
            params, mujoco_model_path=str(mjcf_path), headless=headless, visuals=visuals
        ),
        encoding="utf-8",
    )
    return {"mjcf": mjcf_path, "urdf": urdf_path}
