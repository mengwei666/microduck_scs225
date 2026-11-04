#!/bin/sh
# 8-hour legs-only training: stable stand, forward/back, push recovery.
# 4096 envs x 9000 iters ~= 8 h on the RTX 2070 (measured ~3.2 s/iter).
set -e
cd "$(dirname "$0")"
export CUDA_VISIBLE_DEVICES=0
exec uv run --no-sync train Mjlab-Velocity-Flat-MicroDuck10 \
    --env.scene.num-envs 4096 \
    --agent.max_iterations 9000 \
    --agent.run-name microduck10_8h \
    --agent.logger tensorboard
