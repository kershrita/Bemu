"""Names shared by the MJCF, the URDF and the simulation, so they cannot drift apart."""

from bemu_params import LEG_CODES


def joint_names(leg_code: str) -> tuple[str, str, str]:
    return (f"{leg_code}_coxa_joint", f"{leg_code}_femur_joint", f"{leg_code}_tibia_joint")


def foot_site(leg_code: str) -> str:
    return f"{leg_code}_foot"


def hip_site(leg_code: str) -> str:
    """Site at the leg-frame origin (on the coxa axis)."""
    return f"{leg_code}_hip"


ROOT_JOINT = "root"  # the free joint that lets the body move
BODY = "body"


def joint_order() -> tuple[str, ...]:
    """All 18 joint names: legs in LEG_CODES order, coxa, femur, tibia within each leg."""
    return tuple(name for leg in LEG_CODES for name in joint_names(leg))
