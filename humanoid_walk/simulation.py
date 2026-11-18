"""Sample foot and pelvis motion and solve two-link leg geometry."""

from dataclasses import dataclass
from math import cos, dist, isfinite, pi, sin, sqrt

from .planner import Gait, Pose, angle, foot_pose, plan

Point = tuple[float, float, float]


@dataclass(frozen=True)
class Frame:
    time: float
    pelvis: Pose
    pelvis_z: float
    left_foot: Pose
    right_foot: Pose
    left_z: float
    right_z: float
    left_knee: Point
    right_knee: Point
    support: str


def smooth(u: float) -> float:
    return u * u * (3 - 2 * u)


def interpolate(a: Pose, b: Pose, u: float) -> Pose:
    return Pose(
        a.x + (b.x - a.x) * u,
        a.y + (b.y - a.y) * u,
        angle(a.yaw + angle(b.yaw - a.yaw) * u),
    )


def knee_position(hip: Point, foot: Point, yaw: float, gait: Gait) -> Point:
    length = dist(hip, foot)
    if not abs(gait.thigh - gait.shin) < length <= gait.thigh + gait.shin:
        raise ValueError(f"Leg target is unreachable ({length:.3f} m). Adjust gait geometry.")
    axis = tuple((end - start) / length for start, end in zip(hip, foot))
    forward = (cos(yaw), sin(yaw), 0.0)
    projection = sum(a * b for a, b in zip(axis, forward))
    bend = tuple(f - projection * a for f, a in zip(forward, axis))
    bend_length = dist(bend, (0.0, 0.0, 0.0))
    if bend_length < 1e-9:
        raise ValueError("Cannot determine a forward knee bend for a horizontal leg.")
    along = (gait.thigh**2 - gait.shin**2 + length**2) / (2 * length)
    offset = sqrt(max(0.0, gait.thigh**2 - along**2))
    return tuple(h + along * a + offset * b / bend_length for h, a, b in zip(hip, axis, bend))


def simulate(start: Pose, goal: Pose, gait: Gait = Gait(), hz: float = 50.0) -> list[Frame]:
    if not isfinite(hz) or hz <= 0:
        raise ValueError("Sample rate must be finite and positive.")
    steps = plan(start, goal, gait)
    feet = {side: foot_pose(start, side, gait.stance_width) for side in ("left", "right")}
    samples = max(3, round(gait.step_time * hz))

    def frame(time: float, pelvis: Pose, swing: str = "", height: float = 0.0) -> Frame:
        knees = {}
        heights = {}
        for side, foot in feet.items():
            hip = foot_pose(pelvis, side, gait.stance_width)
            heights[side] = height if side == swing else 0.0
            knees[side] = knee_position(
                (hip.x, hip.y, gait.hip_height), (foot.x, foot.y, heights[side]), pelvis.yaw, gait
            )
        support = "double" if not swing else ("right" if swing == "left" else "left")
        return Frame(
            time, pelvis, gait.hip_height, feet["left"], feet["right"],
            heights["left"], heights["right"], knees["left"], knees["right"], support,
        )

    frames = [frame(0.0, start)]
    for index, step in enumerate(steps):
        stance = feet["right" if step.side == "left" else "left"]
        initial = frames[-1].pelvis
        final = interpolate(stance, step.after, 0.5)
        # Shift the pelvis toward the support foot before lifting the swing foot.
        # Retain the pelvis heading during the lateral weight shift.
        support = Pose(stance.x, stance.y, initial.yaw)
        for sample in range(1, samples + 1):
            u = sample / samples
            swing_u = min(1.0, max(0.0, (u - 0.25) / 0.5))
            feet[step.side] = interpolate(step.before, step.after, smooth(swing_u))
            height = gait.swing_height * sin(pi * swing_u)**2 if 0 < swing_u < 1 else 0.0
            if u < 0.25:
                pelvis = interpolate(initial, support, smooth(u / 0.25))
            elif u <= 0.75:
                pelvis = interpolate(support, Pose(support.x, support.y, final.yaw), smooth(swing_u))
            else:
                pelvis = interpolate(
                    Pose(support.x, support.y, final.yaw), final, smooth((u - 0.75) / 0.25)
                )
            swing = step.side if height > 0 else ""
            frames.append(frame((index + u) * gait.step_time, pelvis, swing, height))
    return frames
