"""Names shared by the MJCF, the URDF and the simulation, so they cannot drift apart."""


def joint_names(leg_code: str) -> tuple[str, str, str]:
    return (f"{leg_code}_coxa_joint", f"{leg_code}_femur_joint", f"{leg_code}_tibia_joint")


def foot_site(leg_code: str) -> str:
    return f"{leg_code}_foot"


def hip_site(leg_code: str) -> str:
    """Site at the leg-frame origin (on the coxa axis)."""
    return f"{leg_code}_hip"
