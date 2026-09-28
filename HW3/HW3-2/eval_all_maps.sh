#!/bin/bash
# Evaluate model.zip on all 5 maps, 3 episodes each (matches grading format)
set -euo pipefail

cd "$(dirname "$0")"

PROLY="Proly.x86_64"
# The PPO policy is a small MLP; CPU inference avoids competing for a busy GPU.
export CUDA_VISIBLE_DEVICES=""
BASE_CMD="uv run --project .. python -m mlgame3d -w 1 -i model_play.py -i hidden -i hidden -i hidden -gp items 0 -gp audio false -gp checkpoint 10 -gp max_time 120"

echo "Evaluating model on all maps (3 episodes each)..."
echo ""

for map_num in 0 1 2 3 4; do
    echo "===== MAP $map_num ====="
    $BASE_CMD -e 3 -gp map $map_num $PROLY
    echo ""
done

echo "Evaluation complete."
