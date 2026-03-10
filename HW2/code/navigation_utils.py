import numpy as np
import cv2
import Simulation.utils as utils
import PathTracking.utils as pt_utils

def pos_int(p):
    return (int(p[0]), int(p[1]))

def render_path(img, nav_pos, way_points, path):
    cv2.circle(img,nav_pos,5,(0.5,0.5,1.0),3)
    for i in range(len(way_points)):    # Draw Way Points
        cv2.circle(img, pos_int(way_points[i]), 3, (1.0,0.4,0.4), 1)
    for i in range(len(path)-1):    # Draw Interpolating Curve
        cv2.line(img, pos_int(path[i]), pos_int(path[i+1]), (1.0,0.4,0.4), 1)
    return img

def render_dynamic_camera_and_minimap(simulator, camera_w, camera_h, path, way_points, nav_pos):
    # --- 1. Dynamic High-Res Camera View ---
    # 10 pixels per meter resolution.
    camera_render_scale = 10.0 
    camera_view = np.ones((camera_h, camera_w, 3), dtype=np.uint8) * 255 # White background
    
    # Helper function to convert world coordinates to camera coordinates
    def world_to_camera(wx, wy):
        # Center of the screen is the ego vehicle
        dx = wx - simulator.state.x
        dy = wy - simulator.state.y
        
        # Scale to pixels
        px = int(dx * camera_render_scale)
        py = int(dy * camera_render_scale)
        
        # Map to screen coordinates (origin at center, Y axis NOT flipped yet)
        screen_x = camera_w // 2 + px
        screen_y = camera_h // 2 + py
        return screen_x, screen_y

    # A. Draw Path (Blue line)
    if path is not None:
        # Only draw path points that are roughly within our view to save CPU
        view_radius_m = (max(camera_w, camera_h) / 2.0) / camera_render_scale + 5.0
        
        # Filtering:
        dists = np.sqrt((path[:,0] - simulator.state.x)**2 + (path[:,1] - simulator.state.y)**2)
        visible_mask = dists < view_radius_m
        
        # We need to draw contiguous segments. A simpler way for python is just iterate and check
        last_pt = None
        for p in path:
            if (p[0] - simulator.state.x)**2 + (p[1] - simulator.state.y)**2 < view_radius_m**2:
                pt = world_to_camera(p[0], p[1])
                if last_pt is not None:
                    cv2.line(camera_view, last_pt, pt, (255, 0, 0), 2)
                last_pt = pt
            else:
                last_pt = None
    
    # B. Draw Waypoints (Red dots)
    if way_points is not None:
        for wp in way_points:
            if (wp[0] - simulator.state.x)**2 + (wp[1] - simulator.state.y)**2 < view_radius_m**2:
                pt = world_to_camera(wp[0], wp[1])
                cv2.circle(camera_view, pt, 3, (0, 0, 255), -1)

    # C. Draw Target Point (Green Circle)
    if nav_pos is not None: # Using nav_pos as the target
        target_pt = world_to_camera(nav_pos[0], nav_pos[1])
        cv2.circle(camera_view, target_pt, 6, (0, 255, 0), 2)
        
    # D. Draw Trajectory History
    if len(simulator.record) > 1:
        color = (0, 97, 255) # Orange
        last_pt = None
        # Draw last 1000 points max to prevent lag
        start_idx = max(0, len(simulator.record) - 1000)
        for i in range(start_idx, len(simulator.record)):
            rec = simulator.record[i]
            if (rec[0] - simulator.state.x)**2 + (rec[1] - simulator.state.y)**2 < view_radius_m**2:
                pt = world_to_camera(rec[0], rec[1])
                if last_pt is not None:
                    cv2.line(camera_view, last_pt, pt, color, 2)
                last_pt = pt
            else:
                last_pt = None
                
    # E. Draw Ego Vehicle (Center of screen) using its native intricate render method!
    old_x, old_y = simulator.state.x, simulator.state.y
    old_scale = simulator.render_scale
    old_record = simulator.record
    
    simulator.state.x = (camera_w // 2) / camera_render_scale
    simulator.state.y = (camera_h // 2) / camera_render_scale
    
    # Just scale up the car for visualization while the simulation is in metric space
    simulator.render_scale = camera_render_scale
    simulator.record = [] # Hide history native rendering, we already drew it relative to camera
    
    if hasattr(simulator, 'car_w'):
        simulator.car_box = utils.compute_car_box(simulator.car_w, simulator.car_f, simulator.car_r, simulator.state.pose())
        
    camera_view = simulator.render(camera_view)
    
    # Restore true state
    simulator.state.x, simulator.state.y = old_x, old_y
    simulator.render_scale = old_scale
    simulator.record = old_record
    if hasattr(simulator, 'car_w'):
        simulator.car_box = utils.compute_car_box(simulator.car_w, simulator.car_f, simulator.car_r, simulator.state.pose())
        
        
    # F. Draw Wind Indicator (if applicable)
    if hasattr(simulator, 'wind_mag') and simulator.wind_mag > 0.0:
        wind_mag = simulator.wind_mag
        wind_angle = simulator.wind_angle
        # Arrow length proportional to magnitude (e.g., 5 pixels per 1 m/s^2) for small arrows
        arrow_len = int(wind_mag * 10) + 5
        if arrow_len > 0:
            dx = int(arrow_len * np.cos(np.deg2rad(wind_angle)))
            dy = int(arrow_len * np.sin(np.deg2rad(wind_angle)))
            
            # Subtle semi-transparent teal color for the arrows
            base_color = (180, 180, 50) # Light Teal / Cyan
            overlay = camera_view.copy()
            
            # Draw a grid of small wind arrows
            grid_spacing_x = int(abs(dy) * 0.5 + 50)
            grid_spacing_y = int(abs(dx) * 0.5 + 50)
            for gy in range(40, camera_h, grid_spacing_y):
                for gx in range(40, camera_w, grid_spacing_x):
                    cv2.arrowedLine(overlay, (gx, gy), (gx + dx, gy + dy), base_color, 1, tipLength=0.2, line_type=cv2.LINE_AA)
            
            # Blend the arrows into the main camera view with 40% opacity
            alpha = 0.4
            camera_view = cv2.addWeighted(overlay, alpha, camera_view, 1 - alpha, 0)
        
    # Now flip the entire camera view vertically so that +Y goes Up!
    camera_view = cv2.flip(camera_view, 0)
    
    # Draw text overlay after flip
    if hasattr(simulator, 'wind_mag') and simulator.wind_mag > 0.0:
        # Create a sleek dark background pill for the text
        text_str = f"WIND: {simulator.wind_mag:.1f} m/s^2"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        (text_w, text_h), baseline = cv2.getTextSize(text_str, font, font_scale, thickness)
        
        box_x, box_y = camera_w - text_w - 40, 20
        box_w, box_h = text_w + 20, text_h + 20
        
        # Semi-transparent dark pill background
        overlay = camera_view.copy()
        cv2.rectangle(overlay, (box_x, box_y), (box_x + box_w, box_y + box_h), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.6, camera_view, 0.4, 0, camera_view)
        
        # Draw teal text on top
        teal = (200, 200, 50)
        cv2.putText(camera_view, text_str, (box_x + 10, box_y + text_h + 10), font, font_scale, teal, thickness, cv2.LINE_AA)
    
    
    # --- 2. Create Minimap directly on small canvas ---
    minimap_h = 200
    
    if path is not None:
        min_mx, max_mx = np.min(path[:,0]), np.max(path[:,0])
        min_my, max_my = np.min(path[:,1]), np.max(path[:,1])
        
        track_w = max_mx - min_mx
        track_h = max_my - min_my
        
        # Proportional Width formulation
        # Protect against division by zero 
        track_h = max(1.0, track_h) 
        minimap_w = int(minimap_h * (track_w / track_h))
        # Optional: clamp minimap width so it doesn't get ridiculously wide for straight lines
        minimap_w = np.clip(minimap_w, 50, 400)
        
        minimap = np.ones((minimap_h, minimap_w, 3), dtype=np.uint8) * 255
        
        mm_margin = 10
        scale_mx = (minimap_w - 2 * mm_margin) / max(1.0, track_w)
        scale_my = (minimap_h - 2 * mm_margin) / max(1.0, track_h)
        mm_scale = min(scale_mx, scale_my)
        
        # Calculate centering offsets
        offset_x = (minimap_w - 2 * mm_margin - track_w * mm_scale) / 2
        offset_y = (minimap_h - 2 * mm_margin - track_h * mm_scale) / 2
        
        
        def world_to_minimap(wx, wy):
            px = int((wx - min_mx) * mm_scale + mm_margin + offset_x)
            py = (minimap_h - 1) - int((wy - min_my) * mm_scale + mm_margin + offset_y)
            # Clip safely to keep lines fully entirely inside the canvas avoiding black border overlap
            px = int(np.clip(px, 2, minimap_w - 3))
            py = int(np.clip(py, 2, minimap_h - 3))
            return px, py
            
        mm_path_pts = []
        for p in path:
            mm_path_pts.append(world_to_minimap(p[0], p[1]))
        mm_path_pts = np.array(mm_path_pts, dtype=np.int32)
        cv2.polylines(minimap, [mm_path_pts], isClosed=False, color=(255, 0, 0), thickness=1)
        
        # Draw Historical Trajectory on Minimap
        if len(simulator.record) > 1:
            # Take up to the last 2000 points to avoid lag if simulation runs a long time
            start_idx = max(0, len(simulator.record) - 2000)
            mm_history_pts = []
            for i in range(start_idx, len(simulator.record)):
                rec = simulator.record[i]
                mm_history_pts.append(world_to_minimap(rec[0], rec[1]))
            mm_history_pts = np.array(mm_history_pts, dtype=np.int32)
            # Bright Orange color for history
            cv2.polylines(minimap, [mm_history_pts], isClosed=False, color=(0, 165, 255), thickness=1)
        
        ego_mm_pt = world_to_minimap(simulator.state.x, simulator.state.y)
        cv2.circle(minimap, ego_mm_pt, 4, (0, 0, 255), -1)
        
    else:
        # Fallback if no path
        minimap_w = 200
        minimap = np.ones((minimap_h, minimap_w, 3), dtype=np.uint8) * 255

    cv2.rectangle(minimap, (0, 0), (minimap_w-1, minimap_h-1), (0,0,0), 2)
    camera_view[10:10+minimap_h, camera_w-minimap_w-10:camera_w-10] = minimap
    return camera_view

def render_velocity_plot(v_hist, v_ref_hist, w, h):
    plot = np.ones((h, w, 3), dtype=np.uint8) * 30 # Dark background
    
    if len(v_hist) < 2:
        return plot
        
    # Max velocity for scaling (e.g. 50 m/s max)
    max_v = max(50.0, max(v_ref_hist) * 1.2, max(v_hist) * 1.2)
    min_v = 0.0
    
    # helper to convert v to y pixel
    def to_y(v):
        return int(h - 10 - (v - min_v) / (max_v - min_v) * (h - 20))
        
    # Draw reference grid lines
    for v_grid in range(0, int(max_v)+1, 10):
        y_grid = to_y(v_grid)
        cv2.line(plot, (0, y_grid), (w, y_grid), (80, 80, 80), 1)
        cv2.putText(plot, f"{v_grid}m/s", (5, y_grid-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        
    step_x = w / 300.0
    
    pts_v = []
    pts_vref = []
    
    for i, (v, v_ref) in enumerate(zip(v_hist, v_ref_hist)):
        x = int(i * step_x)
        pts_v.append((x, to_y(v)))
        pts_vref.append((x, to_y(v_ref)))
        
    # Draw v_ref (Green)
    pts_vref = np.array(pts_vref, dtype=np.int32)
    cv2.polylines(plot, [pts_vref], isClosed=False, color=(0, 255, 0), thickness=2)
    
    # Draw actual v (Cyan)
    pts_v = np.array(pts_v, dtype=np.int32)
    cv2.polylines(plot, [pts_v], isClosed=False, color=(255, 255, 0), thickness=2)
    
    # Legend
    cv2.putText(plot, f"Target v_ref: {v_ref_hist[-1]:.1f}", (w - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(plot, f"Actual v: {v_hist[-1]:.1f}", (w - 200, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    return plot

def evaluate_and_draw_metrics(simulator, path, nav_current_idx, cte_history, has_finished, sim_ticks, camera_view):
    """Calculates cross-track error to the path and overlays metrics HUD onto the camera view."""
    if path is not None and not has_finished:
        nav_current_idx, _ = pt_utils.search_nearest_local(path, (simulator.state.x, simulator.state.y), nav_current_idx, lookahead=50)
        
        if nav_current_idx < len(path) - 1:
            p_i, p_i1 = path[nav_current_idx][:2], path[nav_current_idx + 1][:2]
        else:
            p_i, p_i1 = path[nav_current_idx - 1][:2], path[nav_current_idx][:2]
            
        vec_path = p_i1 - p_i
        vec_car = np.array([simulator.state.x, simulator.state.y]) - p_i
        
        path_len = np.linalg.norm(vec_path)
        if path_len > 1e-5:
            cross_2d = vec_path[0] * vec_car[1] - vec_path[1] * vec_car[0]
            cte = abs(cross_2d / path_len)
        else:
            cte = np.linalg.norm(vec_car)
        cte_history.append(cte)
        
        if nav_current_idx == len(path) - 1 and not has_finished:
            total_time = sim_ticks * simulator.model.dt
            print(f"\n\n{'='*40}")
            print(f"--- Simulation Finished ---")
            print(f"Total Elapsed Time: {total_time:.2f} seconds")
            print(f"Average Cross-Track Error: {np.mean(cte_history):.4f} meters")
            print(f"{'='*40}\n")
            has_finished = True
            
    if len(cte_history) > 0:
        current_time = sim_ticks * simulator.model.dt
        cv2.putText(camera_view, f"Time: {current_time:.1f} s", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(camera_view, f"CTE: {cte_history[-1]:.3f} m", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(camera_view, f"Avg CTE: {np.mean(cte_history):.3f} m", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
        
    return nav_current_idx, has_finished
