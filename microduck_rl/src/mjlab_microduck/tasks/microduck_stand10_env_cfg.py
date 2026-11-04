"""Stand-only task for the legs-only 10-DoF robot.

Zero velocity everywhere, every env in standing mode, no pushes. The point is a
stable HOME pose that can be loaded into robotd's `stand` slot while a separate
walk policy is trained or tested.
"""

from __future__ import annotations

from mjlab.envs import ManagerBasedRlEnvCfg

from mjlab_microduck.tasks.microduck_velocity10_env_cfg import (
    MicroduckRlCfg,
    make_microduck_velocity10_env_cfg,
)


def make_microduck_stand10_env_cfg(play: bool = False, rough: bool = False) -> ManagerBasedRlEnvCfg:
    cfg = make_microduck_velocity10_env_cfg(play=play, rough=rough)

    twist = cfg.commands["twist"]
    twist.ranges.lin_vel_x = (0.0, 0.0)
    twist.ranges.lin_vel_y = (0.0, 0.0)
    twist.ranges.ang_vel_z = (0.0, 0.0)
    twist.rel_standing_envs = 1.0
    twist.rel_turn_in_place_envs = 0.0

    # No pushes while learning to stand still.
    cfg.events.pop("push_robot", None)
    cfg.curriculum.pop("push_magnitude", None)
    cfg.curriculum.pop("standing_envs", None)

    # Standing is the whole objective.
    cfg.rewards["track_linear_velocity"].weight = 0.0
    cfg.rewards["track_angular_velocity"].weight = 0.0
    cfg.rewards["air_time"].weight = 0.0
    cfg.rewards["upright"].weight = 4.0
    cfg.rewards["pose"].weight = 2.0
    cfg.rewards["action_rate_l2"].weight = -0.5
    return cfg


__all__ = ["MicroduckRlCfg", "make_microduck_stand10_env_cfg"]
