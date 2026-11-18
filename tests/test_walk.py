from math import dist, pi
import unittest

from humanoid_walk.planner import Gait, Pose, angle, foot_pose, plan
from humanoid_walk.simulation import simulate


class WalkTest(unittest.TestCase):
    def test_goal_poses_and_final_stance(self):
        cases = [
            (Pose(0, 0), Pose(1.5, 0.5, 1.57)),
            (Pose(0, 0), Pose(-0.31, 0, -1.2)),
            (Pose(2, 3, -2.9), Pose(2, 3, 2.9)),
            (Pose(0.2, -0.3, 3.1), Pose(0.205, -0.3, 0.6)),
            (Pose(0, 0), Pose(0, 0, 2 * pi)),
        ]
        gait = Gait()
        for start, goal in cases:
            with self.subTest(start=start, goal=goal):
                last = simulate(start, goal, gait)[-1]
                for actual, expected in [
                    (last.pelvis, goal),
                    (last.left_foot, foot_pose(goal, "left", gait.stance_width)),
                    (last.right_foot, foot_pose(goal, "right", gait.stance_width)),
                ]:
                    self.assertAlmostEqual(actual.x, expected.x)
                    self.assertAlmostEqual(actual.y, expected.y)
                    self.assertAlmostEqual(angle(actual.yaw - expected.yaw), 0)
                self.assertEqual(last.support, "double")
                self.assertEqual((last.left_z, last.right_z), (0, 0))

    def test_support_foot_and_leg_lengths(self):
        start, goal, gait = Pose(0, 0), Pose(0.7, -0.3, 0.9), Gait()
        steps = plan(start, goal, gait)
        self.assertTrue(all(a.side != b.side for a, b in zip(steps, steps[1:])))
        frames = simulate(start, goal, gait)
        self.assertEqual({frame.support for frame in frames}, {"left", "right", "double"})
        for previous, frame in zip(frames, frames[1:]):
            self.assertGreater(frame.time, previous.time)
            for side in ("left", "right"):
                foot = getattr(frame, f"{side}_foot")
                height = getattr(frame, f"{side}_z")
                hip = foot_pose(frame.pelvis, side, gait.stance_width)
                knee = getattr(frame, f"{side}_knee")
                self.assertGreaterEqual(height, 0)
                self.assertLessEqual(height, gait.swing_height)
                self.assertAlmostEqual(dist((hip.x, hip.y, frame.pelvis_z), knee), gait.thigh)
                self.assertAlmostEqual(dist(knee, (foot.x, foot.y, height)), gait.shin)
                if frame.support == side:
                    self.assertEqual(height, 0)
                    self.assertEqual(foot, getattr(previous, f"{side}_foot"))
                    self.assertEqual((frame.pelvis.x, frame.pelvis.y), (foot.x, foot.y))

    def test_unreachable_geometry_fails(self):
        with self.assertRaisesRegex(ValueError, "unreachable"):
            simulate(Pose(0, 0), Pose(1, 0), Gait(hip_height=0.9))


if __name__ == "__main__":
    unittest.main()
