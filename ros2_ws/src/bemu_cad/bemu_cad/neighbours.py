"""The neighbouring-leg rule found by the CAD check (fit.neighbour_collisions), kept free of the CAD
library so the gait tests can use it anywhere."""

# (leg a, leg b, sign): b is the next leg round the body; sign is +1 on the left side and -1 on the
# right, so that a positive "toward" angle swings a leg toward its neighbour on either side.
NEIGHBOURS = (("lf", "lm", 1), ("lm", "lr", 1), ("rf", "rm", -1), ("rm", "rr", -1))
FREE_TOWARD_SUM = 45.0  # degrees: the largest sum of the two toward angles verified free (docs/08)


def toward_sum(angles: dict, a: str, b: str, sign: int) -> float:
    """Degrees by which legs a and b together swing toward each other (angles: coxa radians by leg
    code, or objects with a `.coxa`)."""
    import math

    def coxa(code):
        value = angles[code]
        return value.coxa if hasattr(value, "coxa") else value

    return math.degrees(sign * (coxa(a) - coxa(b)))
