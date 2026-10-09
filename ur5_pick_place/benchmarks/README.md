# Planning experiment — 2026-10-04

These archived measurements are from the earlier whole-task optimizer. The
current optimizer searches every arm and gripper stage independently, so these
figures must not be used as a performance claim for the current per-stage
selection.

Five consecutive trials used the default optimizer: one baseline followed by
four additional OMPL candidates, the existing Pilz LIN descent, unchanged
eight-stage targets, 0.02 m grasp clearance, 0.10 m approach height, and arm
velocity/acceleration scaling of 0.2. PTP alternatives were disabled.

| Trial | Baseline motion (s) | Selected motion (s) | Search + measurement wall time (s) |
| --- | ---: | ---: | ---: |
| 1 | 42.18 | 33.06 | 6.30 |
| 2 | 47.59 | 38.16 | 6.55 |
| 3 | 37.78 | 34.17 | 14.99 |
| 4 | 37.56 | 37.56 | 11.02 |
| 5 | 41.14 | 33.06 | 12.05 |

The fourth trial retained its baseline because no alternative improved the
comparison without worsening any of its four objectives. Median per-trial
reductions, including that fallback, were:

| Measurement | Median reduction |
| --- | ---: |
| Planned motion duration | 19.6% |
| Arm joint travel | 22.1% |
| Sampled tool travel | 6.1% |
| Sampled acceleration variation | 11.5% |

These are **planning measurements**, not observed robot or Gazebo motion.
Smoothness uses the acceleration-variation proxy defined in the package
README. Motion duration excludes controller overhead and grasp confirmation.
Additional search time can outweigh the motion-time saving for a one-off
task, and the optimizer currently searches again on every invocation.
OMPL is stochastic; five trials do not establish a guaranteed speedup.

The validation environment used ROS 2 Humble and MoveIt move_group
`2.5.9-1jammy.20260326.013255`, in a private ROS domain (74), localhost only.
It loaded the workspace's unmodified URDF, SRDF, kinematics, planning
pipelines, and scene publisher. A temporary joint-state publisher held all
joints at the saved straight/open states, including mimic joints. There was
no Gazebo instance, active robot controller, or execution request. The
temporary planning processes were stopped after validation.

The local tool-distance FK implementation was cross-checked against MoveIt's
`/compute_fk` at the straight, pick, and place joint states. Maximum position
disagreement was approximately `5.1e-16 m`.

An earlier exploratory PTP trial was excluded from this five-trial series.
MoveIt rejected PTP because arm acceleration limits are not explicitly set;
the optimizer kept valid OMPL alternatives. This is why PTP search is opt-in,
and no global limits or planner configuration were changed.

The [recorded measurements](2026-10-04-planning.json) include the settings,
initial joints, all candidate totals, selection decisions, planning times,
and per-trial percentage changes. Repeat the planning comparison with the
commands in the [package README](../README.md#opt-in-trajectory-optimization)
against an idle, unchanged scene. A fresh stochastic run can produce
different results.
