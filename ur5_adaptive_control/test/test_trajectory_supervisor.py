#!/usr/bin/env python3
"""Unit and Integration Tests for Phase 8: PayloadAwareTrajectorySupervisor.

Tests:
1. Profile selection logic (unloaded, loaded, exact mass, continuous interpolation).
2. Fallback selection on unverified / unphysical masses.
3. High-precision mode selection.
4. Trajectory retiming mathematical invariants (100% waypoint and Cartesian geometry preservation).
5. Monotonic timestamps, boundary continuity, and limit validation.
6. Negative tests (rejection of invalid, non-monotonic, or out-of-limit trajectories).
7. Full path file retiming and format preservation.
"""

import copy
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if (ROOT / "ur5_adaptive_control" / "scripts").is_dir():
    sys.path.insert(0, str(ROOT / "ur5_adaptive_control" / "scripts"))
if (ROOT / "ur5_data_logger").is_dir():
    sys.path.insert(0, str(ROOT / "ur5_data_logger"))

from trajectory_supervisor import PayloadAwareTrajectorySupervisor, ScalingProfile
from ur5_data_logger.fk import VerifiedFK
from ur5_data_logger.schema import JOINTS


class TestTrajectorySupervisor(unittest.TestCase):

    def setUp(self):
        self.supervisor = PayloadAwareTrajectorySupervisor()
        self.hp_supervisor = PayloadAwareTrajectorySupervisor(high_precision_mode=True)
        self.path_file = ROOT / "path01"
        self.robot_urdf = ROOT / "experiment_logs" / "test_phase7_1_0kg_full" / "robot.urdf"
        if self.robot_urdf.is_file():
            self.fk = VerifiedFK(self.robot_urdf.read_text(), "base_link", "tool0", JOINTS)
        else:
            self.fk = None

    def test_unloaded_segment_selection(self):
        """Unloaded segments must always receive nominal unloaded profile regardless of mass."""
        for seg in ["straight_to_prepick", "prepick_to_pick", "place_to_straight", "path/straight_to_prepick"]:
            for mass in [0.0, 0.3, 0.5, 1.0, 3.0, 99.0]:
                prof = self.supervisor.select_profile(mass, seg, "adaptive_enabled")
                self.assertFalse(prof.is_fallback)
                self.assertAlmostEqual(prof.velocity_scale, 0.20, places=3)
                self.assertAlmostEqual(prof.acceleration_scale, 0.20, places=3)
                self.assertIn("Unloaded segment", prof.reason)

    def test_loaded_segment_verified_masses(self):
        """Loaded segment (pick_to_place) must select verified Pareto profiles for verified masses."""
        expected = {
            0.30: (0.25, 0.25),
            0.50: (0.22, 0.22),
            0.75: (0.18, 0.18),
            1.00: (0.15, 0.15),
            3.00: (0.10, 0.10),
        }
        for mass, (exp_v, exp_a) in expected.items():
            prof = self.supervisor.select_profile(mass, "pick_to_place", "adaptive_enabled")
            self.assertFalse(prof.is_fallback, f"Mass {mass} kg should not trigger fallback")
            self.assertAlmostEqual(prof.velocity_scale, exp_v, places=2)
            self.assertAlmostEqual(prof.acceleration_scale, exp_a, places=2)

    def test_fallback_on_unverified_masses(self):
        """Negative or out-of-range masses must safely trigger conservative fallback."""
        bad_masses = [-1.0, -0.01, 4.0, 10.0, float("nan"), float("inf")]
        for mass in bad_masses:
            prof = self.supervisor.select_profile(mass, "pick_to_place", "adaptive_enabled")
            self.assertTrue(prof.is_fallback, f"Mass {mass} must trigger fallback")
            self.assertAlmostEqual(prof.velocity_scale, 0.10, places=2)
            self.assertAlmostEqual(prof.acceleration_scale, 0.10, places=2)

    def test_force_fallback_constraint(self):
        """Explicit force_fallback constraint must trigger fallback even for verified masses."""
        prof = self.supervisor.select_profile(1.0, "pick_to_place", "adaptive_enabled", {"force_fallback": True})
        self.assertTrue(prof.is_fallback)
        self.assertAlmostEqual(prof.velocity_scale, 0.10, places=2)

    def test_high_precision_profile_selection(self):
        """High precision supervisor selects conservative profile aiming for sub-millimeter peak error."""
        prof = self.hp_supervisor.select_profile(1.0, "pick_to_place", "adaptive_enabled")
        self.assertLess(prof.velocity_scale, 0.15)
        self.assertLess(prof.acceleration_scale, 0.15)
        self.assertIn("High-Precision", prof.reason)

    def test_waypoint_geometry_preservation(self):
        """Retimed trajectory points must preserve joint configurations and 3D Cartesian coordinates to machine precision."""
        sample_pts = [
            (0.0, [0.0, -1.5, 0.0, -1.5, 0.0, 0.0], [0.0]*6, [0.0]*6),
            (0.1, [0.01, -1.49, 0.01, -1.49, 0.01, 0.01], [0.1]*6, [1.0]*6),
            (0.2, [0.02, -1.48, 0.02, -1.48, 0.02, 0.02], [0.0]*6, [-1.0]*6),
        ]
        retimed = self.supervisor.retime_segment(sample_pts, velocity_scale=0.10, acceleration_scale=0.10)
        self.assertEqual(len(retimed), len(sample_pts))

        for (t_orig, pos_orig, _, _), (t_new, pos_new, _, _) in zip(sample_pts, retimed):
            self.assertEqual(pos_orig, pos_new, "Joint positions must be identically preserved")
            if self.fk:
                xyz_orig = self.fk.xyz(pos_orig)
                xyz_new = self.fk.xyz(pos_new)
                dist = np.linalg.norm(np.array(xyz_orig) - np.array(xyz_new))
                self.assertAlmostEqual(dist, 0.0, places=7, msg="Cartesian coordinates must be identically preserved")

    def test_timestamps_and_derivatives_scaling(self):
        """Time duration expands inversely to velocity scale; velocity scales by sigma and acceleration by sigma^2."""
        sample_pts = [
            (0.0, [0.0]*6, [0.0]*6, [0.0]*6),
            (1.0, [0.1]*6, [0.2]*6, [1.0]*6),
            (2.0, [0.2]*6, [0.0]*6, [0.0]*6),
        ]
        # Base scale is 0.20. Target scale 0.10 -> ratio sigma = 0.5 -> time doubles
        retimed = self.supervisor.retime_segment(sample_pts, velocity_scale=0.10, acceleration_scale=0.10)
        self.assertAlmostEqual(retimed[0][0], 0.0, places=5)
        self.assertAlmostEqual(retimed[1][0], 2.0, places=5)
        self.assertAlmostEqual(retimed[2][0], 4.0, places=5)

        # Velocity scaled by sigma = 0.5: 0.2 * 0.5 = 0.1
        self.assertAlmostEqual(retimed[1][2][0], 0.1, places=5)
        # Acceleration scaled by sigma^2 = 0.25: 1.0 * 0.25 = 0.25
        self.assertAlmostEqual(retimed[1][3][0], 0.25, places=5)

    def test_avoid_scaling_twice(self):
        """Retiming an already scaled trajectory with matching target and base scales must be an identity operation."""
        sample_pts = [
            (0.0, [0.0]*6, [0.0]*6, [0.0]*6),
            (2.0, [0.1]*6, [0.1]*6, [0.25]*6),
            (4.0, [0.2]*6, [0.0]*6, [0.0]*6),
        ]
        # Target scale is 0.10, and base scale is detected as 0.10 (already scaled)
        retimed = self.supervisor.retime_segment(sample_pts, velocity_scale=0.10, acceleration_scale=0.10, base_scale=0.10)
        self.assertEqual(retimed[1][0], 2.0, "Time must not be dilated twice")
        self.assertEqual(retimed[1][2][0], 0.1, "Velocity must not be scaled twice")
        self.assertEqual(retimed[1][3][0], 0.25, "Acceleration must not be scaled twice")

    def test_trajectory_validation_success(self):
        """Well-formed retimed trajectory passes all validation checks."""
        pts = [
            (0.0, [0.0]*6, [0.0]*6, [0.0]*6),
            (1.0, [0.1]*6, [0.1]*6, [0.1]*6),
            (2.0, [0.2]*6, [0.0]*6, [0.0]*6),
        ]
        valid, issues = self.supervisor.validate_trajectory(pts, 0.20, 0.20, "test_seg")
        self.assertTrue(valid, f"Expected valid trajectory, got issues: {issues}")
        self.assertEqual(len(issues), 0)

    def test_trajectory_validation_rejection(self):
        """Supervisor must reject non-monotonic, non-finite, and over-limit trajectories."""
        # Non-monotonic time
        bad_time = [
            (0.0, [0.0]*6, [0.0]*6, [0.0]*6),
            (1.0, [0.1]*6, [0.1]*6, [0.1]*6),
            (0.5, [0.2]*6, [0.0]*6, [0.0]*6),
        ]
        valid, issues = self.supervisor.validate_trajectory(bad_time, 0.20, 0.20, "bad_time")
        self.assertFalse(valid)
        self.assertTrue(any("Non-monotonic" in s for s in issues))

        # Non-zero boundary velocity
        bad_boundary = [
            (0.0, [0.0]*6, [0.5]*6, [0.0]*6),
            (1.0, [0.2]*6, [0.0]*6, [0.0]*6),
        ]
        valid, issues = self.supervisor.validate_trajectory(bad_boundary, 0.20, 0.20, "bad_boundary")
        self.assertFalse(valid)
        self.assertTrue(any("Non-zero" in s for s in issues))

        # Exceeds velocity limit
        over_vel = [
            (0.0, [0.0]*6, [0.0]*6, [0.0]*6),
            (1.0, [0.1]*6, [5.0]*6, [0.0]*6),
            (2.0, [0.2]*6, [0.0]*6, [0.0]*6),
        ]
        valid, issues = self.supervisor.validate_trajectory(over_vel, 0.20, 0.20, "over_vel")
        self.assertFalse(valid)
        self.assertTrue(any("velocity" in s for s in issues))

    def test_full_path_file_retiming(self):
        """Retime full path01 file and verify segment-to-segment boundary continuity."""
        if not self.path_file.is_file():
            self.skipTest("path01 file not found")

        out_path = Path("/tmp/test_path01_retimed")
        try:
            report = self.supervisor.retime_path_file(self.path_file, out_path, payload_mass=1.0)
            self.assertEqual(report["status"], "valid")
            self.assertGreater(report["total_retimed_duration_s"], report["total_original_duration_s"])

            # Verify output file exists and has correct segment headers
            content = out_path.read_text()
            self.assertIn("SEGMENT straight_to_prepick", content)
            self.assertIn("SEGMENT prepick_to_pick", content)
            self.assertIn("SEGMENT pick_to_place", content)
            self.assertIn("SEGMENT place_to_straight", content)
        finally:
            if out_path.is_file():
                out_path.unlink()


if __name__ == "__main__":
    unittest.main()

