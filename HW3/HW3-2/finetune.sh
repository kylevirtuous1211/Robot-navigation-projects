#!/bin/bash
# Fine-tuning script: cycle through maps 1-3 + heavy map 4 training
# model.zip is saved after every policy update, so chaining runs continues training

cd /home/kyle/Desktop/Robot-navigation-projects/HW3/HW3-2

PROLY="Proly.x86_64"
BASE_CMD="uv run --project /home/kyle/Desktop/Robot-navigation-projects/HW3 python -m mlgame3d -ng -ts 10 -w 1 -i rl_play.py -i hidden -i hidden -i hidden -gp items 0 -gp audio false -gp checkpoint 10 -gp max_time 120"

EASY_EPS=200
HARD_EPS=500
NUM_CYCLES=3

echo "Starting fine-tuning: $NUM_CYCLES cycles"
echo "Maps 1-3: $EASY_EPS episodes each | Map 4: ${HARD_EPS}x2 episodes"

for cycle in $(seq 1 $NUM_CYCLES); do
    echo ""
    echo "===== CYCLE $cycle / $NUM_CYCLES ====="

    echo "[Cycle $cycle] MAP 4 (Pothole Island) - $HARD_EPS episodes"
    $BASE_CMD -e $HARD_EPS -gp map 4 $PROLY

    for map_num in 1 2 3; do
        echo "[Cycle $cycle] MAP $map_num - $EASY_EPS episodes"
        $BASE_CMD -e $EASY_EPS -gp map $map_num $PROLY
    done

    echo "[Cycle $cycle] MAP 4 again - $HARD_EPS episodes"
    $BASE_CMD -e $HARD_EPS -gp map 4 $PROLY

    echo "[Cycle $cycle] Complete."
done

echo ""
echo "Fine-tuning complete. Run eval_all_maps.sh to evaluate."
