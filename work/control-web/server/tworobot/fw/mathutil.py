import math


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap(a: float) -> float:
    """Đưa góc về [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def sign(x: float) -> float:
    return 1.0 if x > 0 else -1.0 if x < 0 else 0.0
