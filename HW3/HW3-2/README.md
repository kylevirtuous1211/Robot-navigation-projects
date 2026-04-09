uv run --project /home/kyle/Desktop/Robot-navigation-projects/HW3 \
      python -m mlgame3d Proly.x86_64 \
      --ai model_play.py \
      --episodes 3 \
      --game-param map 4


 uv run python -m mlgame3d -w 1 -i model_play.py -i hidden -i hidden -i hidden -e 10000 -gp items 0 -gp audio false -gp map 3 -gp checkpoint 10 -gp max_time 120 Proly.x86_64

                                                             