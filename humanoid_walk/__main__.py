import argparse
import csv
from pathlib import Path

from .planner import Pose, angle
from .simulation import simulate


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a flat-floor humanoid walking trajectory.")
    parser.add_argument("--start", nargs=3, type=float, default=(0, 0, 0), metavar=("X", "Y", "YAW"))
    parser.add_argument("--goal", nargs=3, required=True, type=float, metavar=("X", "Y", "YAW"))
    parser.add_argument("--csv", type=Path, required=True, help="Output trajectory path.")
    args = parser.parse_args()
    goal = Pose(*args.goal)
    try:
        frames = simulate(Pose(*args.start), goal)
        final = frames[-1].pelvis
        position_error = ((final.x - goal.x)**2 + (final.y - goal.y)**2)**0.5
        yaw_error = abs(angle(final.yaw - goal.yaw))
        if position_error > 1e-8 or yaw_error > 1e-8:
            parser.exit(1, "Trajectory did not reach the goal.\n")
        with args.csv.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow([
                "time", "pelvis_x", "pelvis_y", "pelvis_z", "pelvis_yaw",
                "left_x", "left_y", "left_z", "left_yaw",
                "right_x", "right_y", "right_z", "right_yaw",
                "left_knee_x", "left_knee_y", "left_knee_z",
                "right_knee_x", "right_knee_y", "right_knee_z", "support",
            ])
            for frame in frames:
                writer.writerow([
                    frame.time, frame.pelvis.x, frame.pelvis.y, frame.pelvis_z, frame.pelvis.yaw,
                    frame.left_foot.x, frame.left_foot.y, frame.left_z, frame.left_foot.yaw,
                    frame.right_foot.x, frame.right_foot.y, frame.right_z, frame.right_foot.yaw,
                    *frame.left_knee, *frame.right_knee, frame.support,
                ])
    except (ValueError, OSError) as error:
        parser.exit(1, f"{error}\n")
    print(f"{len(frames)} frames | {frames[-1].time:.2f} s | position error {position_error:.3g} m | yaw error {yaw_error:.3g} rad")


if __name__ == "__main__":
    main()
