import sys
import numpy as np
import cv2

sys.path.append("..")
from Simulation.simulator import Simulator
import Simulation.utils as utils
from Simulation.utils import State, ControlState
from Simulation.kinematic_differential_drive import KinematicModelDifferentialDrive as KinematicModel

# Differential Drive
class SimulatorDifferentialDrive(Simulator):
    def __init__(self,
            lw_range = 34400.0,  
            rw_range = 34400.0,
            dot_lw_range = 8000.0,
            dot_rw_range = 8000.0,
            l = 0.5,     # distance from center to wheel
            wu = 0.3,    # wheel length
            wv = 0.1,    # wheel width
            car_w = 0.8, # car width
            car_f = 0.8, # car front bumper length
            car_r = 0.8, # car rear bumper length
            dt = 0.05,
            render_scale = 10.0 # Multiplier to convert meters to pixels for drawing
        ):
        self.control_type = "diff_drive"
        # Control Constrain
        self.lw_range = lw_range
        self.rw_range = rw_range
        self.dot_lw_range = dot_lw_range
        self.dot_rw_range = dot_rw_range
        
        # Track previous control commands to calculate derivative
        self.p_lw = 0.0
        self.p_rw = 0.0
        
        # Wheel Distance
        self.l = l
        # Wheel size
        self.wu = wu
        self.wv = wv
        # Car size
        self.car_w = car_w
        self.car_f = car_f
        self.car_r = car_r
        # Simulation delta time
        self.dt = dt
        self.render_scale = render_scale
        # In this diff drive model, wheel radius is wu/2
        self.model = KinematicModel(wu/2, l, dt)

        # Initialize State
        self.state = State()
        self.cstate = ControlState(self.control_type, 0.0, 0.0)
        self.car_box = utils.compute_car_box(self.car_w, self.car_f, self.car_r, self.state.pose())
    
    def init_pose(self, pose):
        self.state.update(pose[0], pose[1], pose[2])
        self.cstate = ControlState(self.control_type, 0.0, 0.0)
        self.p_lw = 0.0
        self.p_rw = 0.0
        self.car_box = utils.compute_car_box(self.car_w, self.car_f, self.car_r, self.state.pose())
        self.record = []
        return self.state, {}

    def step(self, command, update_state=True):
        if command is not None:
            # Check Control Command
            self.cstate.lw = command.lw if command.lw is not None else self.cstate.lw
            self.cstate.rw = command.rw if command.rw is not None else self.cstate.rw

        # Angular Acceleration Constrain (Rate of Change)
        # Assuming command is in units of angular velocity (deg/s), dt is in seconds
        lw_diff_limit = self.dot_lw_range * self.dt
        rw_diff_limit = self.dot_rw_range * self.dt
        
        if self.cstate.lw - self.p_lw > lw_diff_limit:
            self.cstate.lw = self.p_lw + lw_diff_limit
        elif self.cstate.lw - self.p_lw < -lw_diff_limit:
            self.cstate.lw = self.p_lw - lw_diff_limit
            
        if self.cstate.rw - self.p_rw > rw_diff_limit:
            self.cstate.rw = self.p_rw + rw_diff_limit
        elif self.cstate.rw - self.p_rw < -rw_diff_limit:
            self.cstate.rw = self.p_rw - rw_diff_limit
            
        # Update previous wheel commands
        self.p_lw = self.cstate.lw
        self.p_rw = self.cstate.rw

        # Angular Velocity Constrain
        if self.cstate.lw > self.lw_range:
            self.cstate.lw = self.lw_range
        elif self.cstate.lw < -self.lw_range:
            self.cstate.lw = -self.lw_range
        if self.cstate.rw > self.rw_range:
            self.cstate.rw = self.rw_range
        elif self.cstate.rw < -self.rw_range:
            self.cstate.rw = -self.rw_range

        # Motion
        state_next = self.model.step(self.state, self.cstate)
        if update_state:
            self.state = state_next
            self.record.append((self.state.x, self.state.y, self.state.yaw))
            self.car_box = utils.compute_car_box(self.car_w, self.car_f, self.car_r, self.state.pose())
        return state_next, {}
    
    def __str__(self):
        return self.state.__str__() + " " + self.cstate.__str__()

    def render(self, img=None):
        if img is None:
            img = np.ones((600,600,3))
        ########## Draw History ##########
        rec_max = 1000
        start = 0 if len(self.record)<rec_max else len(self.record)-rec_max
        # Draw Trajectory
        color = (0/255,97/255,255/255)
        for i in range(start,len(self.record)-1):
            p1 = (int(self.record[i][0]*self.render_scale), int(self.record[i][1]*self.render_scale))
            p2 = (int(self.record[i+1][0]*self.render_scale), int(self.record[i+1][1]*self.render_scale))
            cv2.line(img, p1, p2, color, 1)

        ########## Draw Car ##########
        # Car box
        pts1, pts2, pts3, pts4 = self.car_box
        color = (0,0,0)
        size = 1
        cv2.line(img, tuple((pts1*self.render_scale).astype(int).tolist()), tuple((pts2*self.render_scale).astype(int).tolist()), color, size)
        cv2.line(img, tuple((pts1*self.render_scale).astype(int).tolist()), tuple((pts3*self.render_scale).astype(int).tolist()), color, size)
        cv2.line(img, tuple((pts3*self.render_scale).astype(int).tolist()), tuple((pts4*self.render_scale).astype(int).tolist()), color, size)
        cv2.line(img, tuple((pts2*self.render_scale).astype(int).tolist()), tuple((pts4*self.render_scale).astype(int).tolist()), color, size)
        # Car center & direction
        t1 = utils.rot_pos( self.car_f*1.6, 0, -self.state.yaw) + np.array((self.state.x,self.state.y))
        t2 = utils.rot_pos( 0, self.car_w*0.8, -self.state.yaw) + np.array((self.state.x,self.state.y))
        t3 = utils.rot_pos( 0, -self.car_w*0.8, -self.state.yaw) + np.array((self.state.x,self.state.y))
        
        c_pt = (int(self.state.x*self.render_scale), int(self.state.y*self.render_scale))
        t1_pt = (int(t1[0]*self.render_scale), int(t1[1]*self.render_scale))
        t2_pt = (int(t2[0]*self.render_scale), int(t2[1]*self.render_scale))
        t3_pt = (int(t3[0]*self.render_scale), int(t3[1]*self.render_scale))
        
        cv2.line(img, c_pt, t1_pt, (0,0,255), size)
        cv2.line(img, t2_pt, t3_pt, (255,0,0), size)
        
        ########## Draw Wheels ##########
        w1 = utils.rot_pos( 0, self.l, -self.state.yaw) + np.array((self.state.x,self.state.y))
        w2 = utils.rot_pos( 0,-self.l, -self.state.yaw) + np.array((self.state.x,self.state.y))
        # 4 Wheels
        wu_sc, wv_sc = int(np.clip(self.wu*self.render_scale, 1, a_max=None)), int(np.clip(self.wv*self.render_scale, 1, a_max=None))
        img = utils.draw_rectangle(img,int(w1[0]*self.render_scale),int(w1[1]*self.render_scale),wu_sc,wv_sc,-self.state.yaw)
        img = utils.draw_rectangle(img,int(w2[0]*self.render_scale),int(w2[1]*self.render_scale),wu_sc,wv_sc,-self.state.yaw)
        # Axle
        img = cv2.line(img, tuple((w1*self.render_scale).astype(int).tolist()), tuple((w2*self.render_scale).astype(int).tolist()), (0,0,0), 1)
        return img

if __name__ == "__main__":
    sim = SimulatorDifferentialDrive(render_scale=100.0)
    sim.init_pose((3, 3, 0))
    sim.render()
    cv2.imshow("Car", sim.render())
    cv2.waitKey(0)