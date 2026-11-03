"""Motion planning with linear interpolation and easing."""

import math


# -- easing functions: map t (0-1) to eased t (0-1) --

def linear(t):
    return t


def ease_in(t):
    """Quadratic ease in — slow start."""
    return t * t


def ease_out(t):
    """Quadratic ease out — slow stop."""
    return 1.0 - (1.0 - t) * (1.0 - t)


def ease_in_out(t):
    """Quadratic ease in-out — slow start and stop."""
    if t < 0.5:
        return 2.0 * t * t
    return 1.0 - (-2.0 * t + 2.0) ** 2 / 2.0


def ease_in_cubic(t):
    """Cubic ease in — slower start."""
    return t * t * t


def ease_out_cubic(t):
    """Cubic ease out — slower stop."""
    return 1.0 - (1.0 - t) ** 3


def ease_in_out_cubic(t):
    """Cubic ease in-out — slower start and stop."""
    if t < 0.5:
        return 4.0 * t * t * t
    return 1.0 - (-2.0 * t + 2.0) ** 3 / 2.0


EASING = {
    "linear": linear,
    "ease_in": ease_in,
    "ease_out": ease_out,
    "ease_in_out": ease_in_out,
    "ease_in_cubic": ease_in_cubic,
    "ease_out_cubic": ease_out_cubic,
    "ease_in_out_cubic": ease_in_out_cubic,
}


def interpolate(start, end, steps, easing="ease_in_out"):
    """Generate waypoints along a straight line from start to end.

    start: (x, y, z) starting position
    end:   (x, y, z) ending position
    steps: number of intermediate points (more = smoother)
    easing: name of easing function or callable

    Yields (x, y, z) tuples including the final point but not the start.
    """
    if isinstance(easing, str):
        ease_fn = EASING.get(easing, ease_in_out)
    else:
        ease_fn = easing

    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz

    for i in range(1, steps + 1):
        t = ease_fn(i / steps)
        yield (sx + dx * t, sy + dy * t, sz + dz * t)


def distance(a, b):
    """Euclidean distance between two (x, y, z) points."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def distance_xy(a, b):
    """Horizontal distance between two (x, y, z) points."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.sqrt(dx * dx + dy * dy)


def steps_for_distance(start, end, step_size=5.0, min_steps=5, max_steps=30):
    """Calculate number of steps based on distance and step size (mm)."""
    dist = distance(start, end)
    steps = int(dist / step_size)
    return max(min_steps, min(max_steps, steps))


def hop(start, end, arc_height=30, steps=None, step_size=5.0,
        lift=5.0, easing="ease_in_out"):
    """Generate waypoints for a smooth arc from start to end.

    Lifts straight up first to clear the screen, then arcs to destination.

    start: (x, y, z) starting position
    end: (x, y, z) ending position
    arc_height: mm above the higher of start/end at the peak
    steps: number of waypoints for arc (auto-calculated if None)
    step_size: mm between waypoints when auto-calculating steps
    lift: mm to lift straight up before arcing (clears the screen)
    easing: easing function for XY movement

    Yields (x, y, z) tuples. Does not include start.
    """
    if isinstance(easing, str):
        ease_fn = EASING.get(easing, ease_in_out)
    else:
        ease_fn = easing

    sx, sy, sz = start
    ex, ey, ez = end

    # phase 1: lift straight up to clear the screen
    lifted_z = sz + lift
    yield (sx, sy, lifted_z)

    if steps is None:
        lifted_start = (sx, sy, lifted_z)
        steps = steps_for_distance(lifted_start, end, step_size, min_steps=5, max_steps=30)

    # phase 2: smooth arc from lifted position to end
    for i in range(1, steps + 1):
        raw_t = i / steps
        t = ease_fn(raw_t)
        x = sx + (ex - sx) * t
        y = sy + (ey - sy) * t
        z_base = lifted_z + (ez - lifted_z) * raw_t
        z_arc = 4.0 * arc_height * raw_t * (1.0 - raw_t)
        yield (x, y, z_base + z_arc)
