
navigation.py
 — NewFeature: Replaced the live OpenCV display window with imageio.get_writer() to save the simulation as an MP4 video. This was necessary to run the simulation on a headless server (no display). The output filename is dynamically named as nav_output_{simulator}_{controller}_{track}.mp4. The main loop now automatically terminates when has_finished is set to True by the metrics evaluator, instead of relying on a keyboard Esc event.


 ## Silverstone

 ### pure pursuit

kp=0.1, Lfc=5.0
 ========================================
--- Simulation Finished ---
Total Elapsed Time: 55.50 seconds
Average Cross-Track Error: 0.0265 meters
========================================

kp=1.0, Lfc=5.0
========================================
--- Simulation Finished ---
Total Elapsed Time: 53.40 seconds
Average Cross-Track Error: 2.3545 meters
========================================

kp=0.3, Lfc=5.0
========================================
--- Simulation Finished ---
Total Elapsed Time: 55.25 seconds
Average Cross-Track Error: 0.0836 meters
========================================

kp=0.08, Lfc=5.0
========================================
--- Simulation Finished ---
Total Elapsed Time: 55.10 seconds
Average Cross-Track Error: 0.0278 meters
========================================

kp=0.06, Lfc=5.0
========================================
--- Simulation Finished ---
Total Elapsed Time: 55.30 seconds
Average Cross-Track Error: 0.1754 meters
========================================


### stanley
kp=1.0
========================================
--- Simulation Finished ---
Total Elapsed Time: 55.10 seconds
Average Cross-Track Error: 0.2739 meters
========================================
![alt text](image.png)

kp=5.0
========================================
--- Simulation Finished ---
Total Elapsed Time: 56.00 seconds
Average Cross-Track Error: 0.2815 meters
========================================

kp=0.1
========================================
--- Simulation Finished ---
Total Elapsed Time: 55.20 seconds
Average Cross-Track Error: 0.8002 meters
========================================

### lqr

========================================
--- Simulation Finished ---
Total Elapsed Time: 55.50 seconds
Average Cross-Track Error: 0.2064 meters
========================================

## Monza

### stanley
kp=2.0
========================================
--- Simulation Finished ---
Total Elapsed Time: 47.65 seconds
Average Cross-Track Error: 0.2647 meters
========================================

### lqr
========================================
--- Simulation Finished ---
Total Elapsed Time: 47.50 seconds
Average Cross-Track Error: 0.1316 meters
========================================

### pure pursuit
========================================
--- Simulation Finished ---
Total Elapsed Time: 47.45 seconds
Average Cross-Track Error: 0.0250 meters
========================================


## Suzuka

### pure pursuit
kp=0.1, Lfc=5.0
========================================
--- Simulation Finished ---
Total Elapsed Time: 53.20 seconds
Average Cross-Track Error: 0.0315 meters
========================================

### lqr
========================================
--- Simulation Finished ---
Total Elapsed Time: 53.75 seconds
Average Cross-Track Error: 0.2072 meters
========================================

### stanley
kp=2.0

========================================
--- Simulation Finished ---
Total Elapsed Time: 53.35 seconds
Average Cross-Track Error: 0.2218 meters
========================================
