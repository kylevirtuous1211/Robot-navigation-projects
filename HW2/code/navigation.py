import argparse
import numpy as np
import cv2
import os
import time
import collections
import urllib.request
from Simulation.utils import ControlState
import Simulation.utils as utils
import PathTracking.utils as pt_utils
from trajectory_generator import natural_cubic_spline, adaptive_sampling, uniform_sampling, generate_speed_profile

##############################
# Global Variables
##############################
pose = None
nav_pos = None
way_points = None
path = None
m_cspace = None
set_controller_path = False

##############################
# Navigation
##############################
from navigation_utils import pos_int, render_path, render_dynamic_camera_and_minimap, render_velocity_plot, evaluate_and_draw_metrics

def navigation(args, simulator, controller, planner, start_pose=(100,200,0)):
    global pose, nav_pos, way_points, path, set_controller_path
    window_name = "HW2 Navigation Demo"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    # Resize window to initial size so getWindowImageRect works reliably at start
    cv2.resizeWindow(window_name, 800, 800)
    # Disable mouse click for interactive path planning
    # cv2.setMouseCallback(window_name, mouse_click)
    simulator.init_pose(start_pose)
    command = ControlState(args.simulator, None, None)
    pose = start_pose
    collision_count = 0
    
    # Set controller path to the waypoints
    if way_points is not None:
        controller.set_path(way_points)
        long_controller.set_path(way_points)
        set_controller_path = False
        
    # Histories for plotting
    v_history = collections.deque(maxlen=300)
    v_ref_history = collections.deque(maxlen=300)
    
    # Evaluation Metrics Tracking
    sim_ticks = 0
    cte_history = []
    nav_current_idx = 0
    has_finished = False
    
    # Main Loop
    while(True):
        if not has_finished:
            sim_ticks += 1
            print("\r", simulator, "| Goal:", nav_pos, end="\t")
        # Update State
        simulator.step(command)
        pose = (simulator.state.x, simulator.state.y, simulator.state.yaw)

        target = None
        if path is not None and collision_count == 0:
            if args.simulator == "basic":
                info = {"x": pose[0],
                        "y": pose[1],
                        "yaw": pose[2],
                        "v": simulator.state.v,
                        }
                next_v, target = long_controller.feedback(info)
                next_w = controller.feedback(info)
                command = ControlState("basic", next_v, next_w)
            elif args.simulator == "diff_drive":
                info = {"x": pose[0],
                        "y": pose[1],
                        "yaw": pose[2],
                        "v": simulator.state.v,
                        }
                next_v, target = long_controller.feedback(info)
                next_w = controller.feedback(info)
                # TODO 2.2.2: Map [v, w] to [lw, rw]
                next_lw = 0
                next_rw = 0
                # [end] TODO 2.2.2
                command = ControlState("diff_drive", next_lw, next_rw)
            elif args.simulator == "bicycle":
                info = {"x": pose[0],
                        "y": pose[1],
                        "yaw": pose[2],
                        "v": simulator.state.v,
                        "delta": simulator.cstate.delta,
                        }
                next_a, target = long_controller.feedback(info)
                info["v"] = info["v"] + next_a * simulator.model.dt
                next_delta = controller.feedback(info)
                command = ControlState("bicycle", next_a, next_delta)
            else:
                exit()            
        else:
            command = None

        if target is not None:
            v_ref_history.append(target[4])
        elif len(v_ref_history) > 0:
            v_ref_history.append(v_ref_history[-1])
        else:
            v_ref_history.append(0.0)
            
        v_history.append(simulator.state.v)
        
        _, info = simulator.step(command)        
        camera_view = render_dynamic_camera_and_minimap(simulator, camera_w, camera_h, path, way_points, nav_pos)
        
        # Evaluate and Draw Metrics HUD
        nav_current_idx, has_finished = evaluate_and_draw_metrics(
            simulator, path, nav_current_idx, cte_history, has_finished, sim_ticks, camera_view
        )
        
        # Velocity Plot
        plot_view = render_velocity_plot(v_history, v_ref_history, camera_w, 250)
        final_view = np.vstack((camera_view, plot_view))
        
        # Show the final tracking view
        cv2.imshow(window_name, final_view)
        
        k = cv2.waitKey(1)
        if k == ord('r'):
            simulator.init_state(start_pose)
            sim_ticks = 0
            cte_history = []
            nav_current_idx = 0
            has_finished = False
        if k == 27:
            print()
            break

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--simulator", type=str, default="basic", choices=['basic', 'diff_drive', 'bicycle'], help="basic/diff_drive/bicycle")
    parser.add_argument("-c", "--controller", type=str, default="pid", choices=['pid', 'pure_pursuit', 'stanley', 'lqr'], help="pid/pure_pursuit/stanley/lqr")
    parser.add_argument("-t", "--track", type=str, default="1000mStraight", choices=['400mRunningTrack', '1000mStraight', 'Silverstone', 'Suzuka', 'Monza'], help="Name of track to load")
    parser.add_argument("-lcs", "--lqr_control_state", type=str, default="steering_angle", choices=['steering_angle', 'steering_angular_velocity'], help="control state of LQR control of bicycle model")
    parser.add_argument("-is", "--init_shift", type=float, default=0.0, help="init location shift")
    return parser.parse_args()

def setup_simulator_and_controller(args):
    try:
        if args.simulator == "basic":
            from Simulation.simulator_basic import SimulatorBasic
            simulator = SimulatorBasic()
            from PathTracking.long_controller_vanilla import VanillaLongController
            l_controller = VanillaLongController()
            if args.controller == "pid":
                from PathTracking.controller_pid_basic import ControllerPIDBasic as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "pure_pursuit":
                from PathTracking.controller_pure_pursuit_basic import ControllerPurePursuitBasic as Controller
                controller = Controller(model=simulator.model, Lfc=1)
            elif args.controller == "lqr":
                from PathTracking.controller_lqr_basic import ControllerLQRBasic as Controller
                controller = Controller(model=simulator.model)
            else:
                raise NameError("Unknown controller!!")
        elif args.simulator == "diff_drive":
            from Simulation.simulator_differential_drive import SimulatorDifferentialDrive
            simulator = SimulatorDifferentialDrive()
            from PathTracking.long_controller_vanilla import VanillaLongController
            l_controller = VanillaLongController()
            if args.controller == "pid":
                from PathTracking.controller_pid_basic import ControllerPIDBasic as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "pure_pursuit":
                from PathTracking.controller_pure_pursuit_basic import ControllerPurePursuitBasic as Controller
                controller = Controller(model=simulator.model, Lfc=1)
            elif args.controller == "lqr":
                from PathTracking.controller_lqr_basic import ControllerLQRBasic as Controller
                controller = Controller(model=simulator.model)
            else:
                raise NameError("Unknown controller!!")
        elif args.simulator == "bicycle":
            from Simulation.simulator_bicycle import SimulatorBicycle 
            simulator = SimulatorBicycle()
            from PathTracking.long_controller_pid import PIDLongController
            l_controller = PIDLongController(model=simulator.model, a_range=simulator.a_range, kp=1, ki=0.0001, kd=0.5)
            if args.controller == "pid":
                from PathTracking.controller_pid_bicycle import ControllerPIDBicycle as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "pure_pursuit":
                from PathTracking.controller_pure_pursuit_bicycle import ControllerPurePursuitBicycle as Controller
                controller = Controller(model=simulator.model, Lfc=1)
            elif args.controller == "stanley":
                from PathTracking.controller_stanley_bicycle import ControllerStanleyBicycle as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "lqr":
                from PathTracking.controller_lqr_bicycle import ControllerLQRBicycle as Controller
                controller = Controller(model=simulator.model, control_state=args.lqr_control_state)
            else:
                raise NameError("Unknown controller!!")
        else:
            raise NameError("Unknown simulator!!")
        return simulator, controller, l_controller, None
    except:
        raise

def load_and_process_track(track_name, map_w, map_h, simulator):
    filename = f"tracks/{track_name}.csv"  
    print(f"Loading {track_name} track...")
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    raw_x = data[:, 0]
    raw_y = data[:, 1]
    
    margin = 5
    track_width_m = max(1e-5, np.max(raw_x) - np.min(raw_x))
    track_height_m = max(1e-5, np.max(raw_y) - np.min(raw_y))
    
    scale_x_px_per_m = (map_w - 2 * margin * 10) / track_width_m  
    scale_y_px_per_m = (map_h - 2 * margin * 10) / track_height_m
    
    render_scale = min(scale_x_px_per_m, scale_y_px_per_m)
    simulator.render_scale = render_scale
    
    center_x = (np.max(raw_x) + np.min(raw_x)) / 2.0
    center_y = (np.max(raw_y) + np.min(raw_y)) / 2.0
    
    scaled_x = (raw_x - center_x) + (map_w / 2.0) / render_scale
    scaled_y = (raw_y - center_y) + (map_h / 2.0) / render_scale
    
    t_anchors = np.linspace(0, 1, len(scaled_x))
    t_path = np.linspace(0, 1, 2000)
    path_x = natural_cubic_spline(t_anchors, scaled_x, t_path)
    path_y = natural_cubic_spline(t_anchors, scaled_y, t_path)
    
    v_ref, k = generate_speed_profile(path_x, path_y, max_v=85.0, max_lat_acc=30, max_long_acc=12, max_long_dec=18)
    wp_x, wp_y, wp_v = adaptive_sampling(path_x, path_y, k, v_ref=v_ref, min_ds=2.0, max_ds=10.0, k_gain=200.0)
    
    wp_yaw = np.zeros_like(wp_x)
    for i in range(len(wp_x)-1):
        wp_yaw[i] = np.rad2deg(np.arctan2(wp_y[i+1]-wp_y[i], wp_x[i+1]-wp_x[i]))
    wp_yaw[-1] = wp_yaw[-2]
    
    wp_k = np.zeros_like(wp_x)
    
    path_yaw = np.zeros_like(path_x)
    for i in range(len(path_x)-1):
        path_yaw[i] = np.rad2deg(np.arctan2(path_y[i+1]-path_y[i], path_x[i+1]-path_x[i]))
    path_yaw[-1] = path_yaw[-2]
    
    w_pts = np.vstack((wp_x, wp_y, wp_yaw, wp_k, wp_v)).T
    p = np.vstack((path_x, path_y, path_yaw, k, v_ref)).T
    return w_pts, p

if __name__ == "__main__":
    args = parse_arguments()

    camera_w, camera_h = 800, 800
    map_w, map_h = 2000, 2000 

    simulator, controller, long_controller, planner = setup_simulator_and_controller(args)
    way_points, path = load_and_process_track(args.track, map_w, map_h, simulator)
    
    nav_pos = (int(path[-1][0]), int(path[-1][1]))
    start_yaw = np.rad2deg(np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0]))
    start_pose = (path[0][0] + np.sin(np.deg2rad(start_yaw))*args.init_shift, path[0][1] + np.cos(np.deg2rad(start_yaw))*args.init_shift, start_yaw)
    
    navigation(args, simulator, controller, planner, start_pose=start_pose)