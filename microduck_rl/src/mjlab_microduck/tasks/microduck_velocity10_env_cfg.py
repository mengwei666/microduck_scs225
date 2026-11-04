"""Velocity task for the legs-only 10-DoF robot (no neck/head).

The robot has 10 actuated leg joints: hip_yaw, hip_roll, hip_pitch, knee and
ankle on each side.  The task trains forward/backward velocity tracking with
the standard balance/upright/termination recipe, so the policy both moves and
stays upright.

The shared observation layout keeps the 4-D head command and 6-D body command
slots zero-padded, but the joint terms are 10-D instead of 14-D.  This policy
therefore needs the matching 10-DoF runtime, not the 13/14-DoF robotd build.
"""

from __future__ import annotations

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.managers import CurriculumTermCfg

from mjlab_microduck.robot.microduck_constants import (
    MICRODUCK_WALK_10DOF_ROBOT_CFG,
)
from mjlab_microduck.tasks import mdp as microduck_mdp
from mjlab_microduck.tasks.microduck_velocity_env_cfg import (
    MicroduckRlCfg,
    make_microduck_velocity_env_cfg,
)

# Forward positive, backward negative. Keep it modest for the first gait.
VX_RANGE = (-0.2, 0.3)

# The 4-D head command slot is kept for observation-shape parity, but there is
# no head to track. Zero it and remove the head-specific rewards/curricula.
HEAD_ZERO = ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0))


def make_microduck_velocity10_env_cfg(
    play: bool = False,
    rough: bool = False,
) -> ManagerBasedRlEnvCfg:
    """Build the 10-DoF vx-only velocity environment on the velocity recipe."""
    cfg = make_microduck_velocity_env_cfg(play=play, rough=rough)
    cfg.scene.entities = {"robot": MICRODUCK_WALK_10DOF_ROBOT_CFG}

    twist = cfg.commands["twist"]
    twist.ranges.lin_vel_x = VX_RANGE
    twist.ranges.lin_vel_y = (0.0, 0.0)
    twist.ranges.ang_vel_z = (0.0, 0.0)
    twist.rel_turn_in_place_envs = 0.0
    # A larger fixed standing fraction: this robot is also expected to stand still.
    # The base standing curriculum would override this, so remove it below.
    twist.rel_standing_envs = 0.35

    # Keep the command slot for obs layout, but never ask for head motion.
    cfg.commands["head_pose"].ranges = HEAD_ZERO

    # The base velocity recipe adds these for the 14/13-DoF robot. They
    # reference head joints/bodies that do not exist in the 10-DoF model.
    for name in ("head_pose_tracking", "head_pose_bias"):
        cfg.rewards.pop(name, None)
    for name in ("head_pose_range", "head_pose_bias_weight", "head_com_range",
                 "standing_envs"):
        cfg.curriculum.pop(name, None)
    cfg.events.pop("randomize_head_com", None)

    # Ramp external pushes in after the gait starts forming, then strengthen
    # them. This is the "shove it and it recovers" objective.
    if "push_robot" in cfg.events:
        cfg.curriculum["push_magnitude"] = CurriculumTermCfg(
            func=microduck_mdp.push_curriculum,
            params={
                "event_name": "push_robot",
                "push_stages": [
                    {"step": 0, "velocity_range": {"x": (0.0, 0.0), "y": (0.0, 0.0)}},
                    {"step": 500 * 24, "velocity_range": {"x": (-0.15, 0.15), "y": (-0.15, 0.15)}},
                    {"step": 1500 * 24, "velocity_range": {"x": (-0.25, 0.25), "y": (-0.25, 0.25)}},
                ],
            },
        )
    return cfg


__all__ = ["MicroduckRlCfg", "make_microduck_velocity10_env_cfg"]
