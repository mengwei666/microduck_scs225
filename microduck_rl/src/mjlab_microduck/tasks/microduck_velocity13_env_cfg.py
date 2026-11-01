"""Velocity task for the 13-DoF bench robot — forward and backward only.

The robot has no head-yaw servo (that joint is pinned at zero in the model), so the
policy has 13 actions. This task is a deliberate simplification of the velocity
recipe: every command except forward/backward travel is held at home, so the first
thing trained is plain vx tracking. The full recipe — lateral, turning, head pose —
stays in `microduck_velocity_env_cfg` for the 14-DoF machine.

Kept short on purpose: this is the action to rehearse on a bench before the robot is
assembled, not the final gait.
"""

from __future__ import annotations

from mjlab.envs import ManagerBasedRlEnvCfg

from mjlab_microduck.robot.microduck_constants import MICRODUCK_WALK_13DOF_ROBOT_CFG
from mjlab_microduck.tasks.microduck_velocity_env_cfg import (
    MicroduckRlCfg,
    make_microduck_velocity_env_cfg,
)

# Forward positive, backward negative. Kept modest so a short run can actually
# reach the target instead of chasing a speed the robot never learns.
VX_RANGE = (-0.3, 0.5)

# (neck_pitch, head_pitch, head_yaw, head_roll) deltas from HOME. All zero: the head
# rides along, and the absent head-yaw slot cannot go dead because the joint is pinned.
HEAD_HOLD = ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0))


def make_microduck_velocity13_env_cfg(
    play: bool = False,
    rough: bool = False,
) -> ManagerBasedRlEnvCfg:
    """Build the 13-DoF vx-only velocity environment on top of the velocity recipe."""
    cfg = make_microduck_velocity_env_cfg(play=play, rough=rough)
    cfg.scene.entities = {"robot": MICRODUCK_WALK_13DOF_ROBOT_CFG}

    twist = cfg.commands["twist"]
    twist.ranges.lin_vel_x = VX_RANGE
    twist.ranges.lin_vel_y = (0.0, 0.0)
    twist.ranges.ang_vel_z = (0.0, 0.0)
    twist.rel_turn_in_place_envs = 0.0
    twist.rel_standing_envs = 0.0

    cfg.commands["head_pose"].ranges = HEAD_HOLD
    if "head_pose_range" in cfg.curriculum:
        cfg.curriculum["head_pose_range"].params["range_stages"] = [
            {"step": 0, "ranges": HEAD_HOLD}
        ]
    # The standing fraction curriculum would otherwise turn a third of the batch into
    # stand-still drills; for vx tracking that is wasted experience.
    cfg.curriculum.pop("standing_envs", None)
    return cfg


__all__ = ["MicroduckRlCfg", "make_microduck_velocity13_env_cfg"]
