"""Plan alternating footsteps for a flat, obstacle-free floor."""

from dataclasses import dataclass
from math import atan2, ceil, cos, hypot, isfinite, pi, sin


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Gait:
    stance_width: float = 0.24
    stride: float = 0.20
    yaw_step: float = pi / 12
    step_time: float = 0.65
    swing_height: float = 0.055
    hip_height: float = 0.72
    thigh: float = 0.40
    shin: float = 0.40


@dataclass(frozen=True)
class Step:
    side: str
    before: Pose
    after: Pose


def angle(value: float) -> float:
    return (value + pi) % (2 * pi) - pi


def foot_pose(center: Pose, side: str, width: float) -> Pose:
    lateral = width / 2 if side == "left" else -width / 2
    return Pose(
        center.x - lateral * sin(center.yaw),
        center.y + lateral * cos(center.yaw),
        angle(center.yaw),
    )


def plan(start: Pose, goal: Pose, gait: Gait) -> list[Step]:
    """Turn toward the target, walk straight, then align with its heading."""
    if not all(isfinite(v) for pose in (start, goal) for v in vars(pose).values()):
        raise ValueError("Poses must contain finite values.")
    if any(not isfinite(v) or v <= 0 for v in vars(gait).values()):
        raise ValueError("Gait parameters must be finite and positive.")
    if gait.yaw_step > pi / 4:
        raise ValueError("yaw_step must be at most pi/4.")

    feet = {side: foot_pose(start, side, gait.stance_width) for side in ("left", "right")}
    side = "left"
    steps = []

    def move(center: Pose) -> None:
        nonlocal side
        target = foot_pose(center, side, gait.stance_width)
        steps.append(Step(side, feet[side], target))
        feet[side] = target
        side = "right" if side == "left" else "left"

    def turn(center: Pose, yaw: float) -> None:
        delta = angle(yaw - center.yaw)
        if abs(delta) <= 1e-12:
            return
        count = ceil(abs(delta) / gait.yaw_step)
        for index in range(1, count + 1):
            waypoint = Pose(center.x, center.y, center.yaw + delta * index / count)
            # Bring both feet to each heading before advancing the turn.
            move(waypoint)
            move(waypoint)

    dx, dy = goal.x - start.x, goal.y - start.y
    distance = hypot(dx, dy)
    heading = start.yaw
    if distance > 1e-12:
        heading = atan2(dy, dx)
        turn(start, heading)
        count = ceil(distance / gait.stride)
        for index in range(1, count + 1):
            fraction = index / count
            move(Pose(start.x + dx * fraction, start.y + dy * fraction, heading))
        move(Pose(goal.x, goal.y, heading))

    turn(Pose(goal.x, goal.y, heading), goal.yaw)
    return steps
