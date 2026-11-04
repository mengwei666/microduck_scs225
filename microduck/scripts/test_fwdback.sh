#!/bin/sh
# Forward-then-backward leg test. Run from anywhere; uses legs_config.json
# next to this script. Extra arguments are passed to legs_bringup.py.
set -e
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec python3 "$HERE/legs_bringup.py" walk \
    --direction both \
    --cycles "${CYCLES:-3}" \
    --stride "${STRIDE:-10}" \
    --lift "${LIFT:-12}" \
    --shift "${SHIFT:-4}" \
    --period "${PERIOD:-1.2}" \
    "$@"
