# Robot Navigation and Exploration

Coursework for NTHU's Robotic Navigation and Exploration (spring 2026): classical planning and control, deep reinforcement learning, perception, and a fully autonomous rover in a Unity simulator.
Each assignment folder has its own README; HW1-HW4 include results and the commands that produced them.

| | Assignment | Highlights |
|---|---|---|
| <img src="HW1/assets/map2_rrt_star_output.png" width="320" alt="RRT* path on map 2"> | **[HW1 - Path planning](HW1/README.md)**<br>A* and RRT* on three occupancy maps | In a seeded run, RRT* finds 2-9% shorter paths than grid A* while exploring up to 3.7x as many nodes |
| <img src="HW2/assets/compare_Suzuka.gif" width="320" alt="Pure pursuit, Stanley and LQR on Suzuka"> | **[HW2 - Path tracking control](HW2/README.md)**<br>Pure pursuit, Stanley and LQR on a bicycle model around three F1 circuits | Tuned pure pursuit holds 0.03 m average cross-track error, about 6.5x tighter than LQR and 8.5x tighter than Stanley |
| <img src="HW3/assets/proly_map4.gif" width="320" alt="PPO agent on Proly map 4"> | **[HW3 - Deep RL](HW3/README.md)**<br>PPO agent for the Unity game *Proly* | 10/10 flags on all five maps in the recorded runs; reward rebalancing fixed the map that started at 0/10 |
| <img src="HW4/assets/val_examples.jpg" width="320" alt="YOLO bear and knob detections and bridge and road segmentation on validation frames"> | **[HW4 - Detection and segmentation](HW4/README.md)**<br>YOLO26s trained on 106 hand-labeled sim frames | 0.97 detection mAP50 (bear, knob) and 0.92 mask mAP50 (bridge, road) on a 16-frame validation split |
| <img src="HW4/assets/bridge_far.jpg" width="320" alt="Rover camera view of the Task 2 bridge with a bear on top"> | **[Final Project - Autonomous rover](Final_Project/README.md)**<br>ROS 2 + SLAM + YOLO state machines for three Unity missions | Hands-free missions: fetch a bear back to the start, climb a bridge to grab the bear on top, and press a door lever and drive through |

## Layout

```
HW1/            A* / RRT* planners (+ RL/: PPO navigation experiment)
HW2/            kinematic models, speed profile, pure pursuit / Stanley / LQR
HW3/            HW3-1 PPO path tracking, HW3-2 PPO agent for Proly
HW4/            YOLO detection + segmentation training and the ROS 2 inference nodes
Final_Project/  Docker Compose ROS 2 stack and the per-task mission state machines
tools/          helpers for README media (GIFs, comparison videos, screen recording, link check)
```

## Environment

- Python projects use [uv](https://docs.astral.sh/uv/); each README's Reproduce section has the exact commands (HW3 is a uv project: `uv run --project HW3 ...`).
- HW4 and the Final Project run ROS 2 Humble in Docker; see their READMEs for the stack.
- The Unity simulators (PROS Twin, Proly) come from the course materials and are git-ignored.
- Trained weights and the labeled datasets are also git-ignored; the Roboflow datasets are linked from [`HW4/report.md`](HW4/report.md).
