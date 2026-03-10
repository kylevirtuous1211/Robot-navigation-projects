import sys
import numpy as np
import cv2
import random

sys.path.append("..")
from Simulation.simulator import Simulator
import Simulation.utils as utils
from Simulation.utils import State, ControlState
from Simulation.kinematic_bicycle import KinematicModelBicycle as KinematicModel

class SimulatorBicycle(Simulator):
    def __init__(self,
            v_range = 90.0,
            a_range = (-20.0, 15.0),
            delta_range = 40.0,
            delta_dot_range = 25.0,
            l = 3.5,     # distance between rear and front wheel
            d = 1.8,     # track width
            wu = 0.8,    # wheel length 
            wv = 0.3,    # wheel width
            car_w = 2.0, # car width
            car_f = 4.5, # car front bumper length
            car_r = 1.0, # car rear bumper length
            dt = 0.05,
            render_scale = 10.0 # Multiplier to convert meters to pixels for drawing
        ):
        self.control_type = "bicycle"
        # Control Constrain
        self.a_range = a_range
        self.delta_range = delta_range
        self.delta_dot_range = delta_dot_range
        self.p_delta = 0.0
        # Speed Constrain
        self.v_range = v_range
        # Distance from center to wheel
        self.l = l
        # Wheel Distance
        self.d = d
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
        self.model = KinematicModel(l, dt)

        # Initialize State
        self.state = State()
        self.cstate = ControlState(self.control_type, 0.0, 0.0)
        self.car_box = utils.compute_car_box(self.car_w, self.car_f, self.car_r, self.state.pose())

        # Environmental factors
        self.wind_mag = 0.0
        self.wind_angle = 0.0

    def init_pose(self, pose):
        self.state.update(pose[0], pose[1], pose[2])
        self.cstate = ControlState(self.control_type, 0.0, 0.0)
        self.car_box = utils.compute_car_box(self.car_w, self.car_f, self.car_r, self.state.pose())
        self.record = []
        
        # Roll random wind disturbance on respawn ([0, 5.0] m/s^2, [0, 360] deg)
        # self.wind_mag = random.uniform(0.0, 5.0)
        # self.wind_angle = random.uniform(0.0, 360.0)
        
        return self.state, {}

    def step(self, command, update_state=True):
        if command is not None:
            # Check Control Command
            self.cstate.a = command.a if command.a is not None else self.cstate.a
            self.cstate.delta = command.delta if command.delta is not None else self.cstate.delta

        # Control Constrain
        if self.cstate.a > self.a_range[1]:
            self.cstate.a = self.a_range[1]
        elif self.cstate.a < self.a_range[0]:
            self.cstate.a = self.a_range[0]
        if self.cstate.delta - self.p_delta > self.delta_dot_range:
            self.cstate.delta = self.p_delta + self.delta_dot_range
        elif self.cstate.delta - self.p_delta < -self.delta_dot_range:
            self.cstate.delta = self.p_delta - self.delta_dot_range
        self.p_delta = self.cstate.delta
        if self.cstate.delta > self.delta_range:
            self.cstate.delta = self.delta_range
        elif self.cstate.delta < -self.delta_range:
            self.cstate.delta = -self.delta_range
        
        
        # State Constrain
        if self.state.v > self.v_range:
            self.state.v = self.v_range
        elif self.state.v < -self.v_range:
            self.state.v = -self.v_range
        
        # Motion
        # If update_state is False, we calculate the next state without applying it
        state_next = self.model.step(self.state, self.cstate)
        
        # # Apply Wind Disturbance only if the car is moving (overcoming static friction)
        # # This allows the car to come to a complete stop at the end of the track
        # if abs(self.state.v) > 1.5:
        #     # Apply Wind Disturbance (Drift)
        #     dx_wind = 0.5 * self.wind_mag * np.cos(np.deg2rad(self.wind_angle)) * (self.dt ** 2)
        #     dy_wind = 0.5 * self.wind_mag * np.sin(np.deg2rad(self.wind_angle)) * (self.dt ** 2)
            
        #     state_next.x += dx_wind
        #     state_next.y += dy_wind
            
        #     # Apply Wind Disturbance to longitudinal velocity (v)
        #     # Project the wind vector onto the car's forward axis
        #     wind_v_proj = self.wind_mag * np.cos(np.deg2rad(self.wind_angle - self.state.yaw))
        #     state_next.v += wind_v_proj * self.dt
        
        if update_state:
            self.state = state_next
            self.car_box = utils.compute_car_box(self.car_w, self.car_f, self.car_r, self.state.pose())
            self.record.append(self.state.pose())

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
        for i in range(start,len(self.record)-1):
            color = (0/255,97/255,255/255)
            p1 = (int(self.record[i][0]*self.render_scale), int(self.record[i][1]*self.render_scale))
            p2 = (int(self.record[i+1][0]*self.render_scale), int(self.record[i+1][1]*self.render_scale))
            cv2.line(img, p1, p2, color, 1)

        ########## Draw Car ##########
        pts1, pts2, pts3, pts4 = self.car_box
        color = (0,0,0)
        size = 1
        cv2.line(img, tuple((pts1*self.render_scale).astype(int).tolist()), tuple((pts2*self.render_scale).astype(int).tolist()), color, size)
        cv2.line(img, tuple((pts1*self.render_scale).astype(int).tolist()), tuple((pts3*self.render_scale).astype(int).tolist()), color, size)
        cv2.line(img, tuple((pts3*self.render_scale).astype(int).tolist()), tuple((pts4*self.render_scale).astype(int).tolist()), color, size)
        cv2.line(img, tuple((pts2*self.render_scale).astype(int).tolist()), tuple((pts4*self.render_scale).astype(int).tolist()), color, size)
        # Car center & direction
        t1 = utils.rot_pos( self.car_f*0.8, 0, -self.state.yaw) + np.array((self.state.x,self.state.y))
        t2 = utils.rot_pos( 0, self.car_w*0.8, -self.state.yaw) + np.array((self.state.x,self.state.y))
        t3 = utils.rot_pos( 0, -self.car_w*0.8, -self.state.yaw) + np.array((self.state.x,self.state.y))
        # 轉成 pixel
        c_pt = (int(self.state.x*self.render_scale), int(self.state.y*self.render_scale))
        t1_pt = (int(t1[0]*self.render_scale), int(t1[1]*self.render_scale))
        t2_pt = (int(t2[0]*self.render_scale), int(t2[1]*self.render_scale))
        t3_pt = (int(t3[0]*self.render_scale), int(t3[1]*self.render_scale))
        cv2.line(img, c_pt, t1_pt, (0,0,255), size)
        cv2.line(img, t2_pt, t3_pt, (255,0,0), size)
        
        ########## Draw Wheels ##########
        w1 = utils.rot_pos( self.l, self.d/2, -self.state.yaw) + np.array((self.state.x,self.state.y))
        w2 = utils.rot_pos( self.l,-self.d/2, -self.state.yaw) + np.array((self.state.x,self.state.y))
        w3 = utils.rot_pos( 0, self.d/2, -self.state.yaw) + np.array((self.state.x,self.state.y))
        w4 = utils.rot_pos( 0,-self.d/2, -self.state.yaw) + np.array((self.state.x,self.state.y))
        # 4 Wheels
        wu_sc, wv_sc = int(np.clip(self.wu*self.render_scale, 1, a_max=None)), int(np.clip(self.wv*self.render_scale, 1, a_max=None))
        img = utils.draw_rectangle(img,int(w1[0]*self.render_scale),int(w1[1]*self.render_scale),wu_sc,wv_sc,-self.state.yaw-self.cstate.delta)
        img = utils.draw_rectangle(img,int(w2[0]*self.render_scale),int(w2[1]*self.render_scale),wu_sc,wv_sc,-self.state.yaw-self.cstate.delta)
        img = utils.draw_rectangle(img,int(w3[0]*self.render_scale),int(w3[1]*self.render_scale),wu_sc,wv_sc,-self.state.yaw)
        img = utils.draw_rectangle(img,int(w4[0]*self.render_scale),int(w4[1]*self.render_scale),wu_sc,wv_sc,-self.state.yaw)
        # Axle
        img = cv2.line(img, tuple((w1*self.render_scale).astype(int).tolist()), tuple((w2*self.render_scale).astype(int).tolist()), (0,0,0), 1)
        img = cv2.line(img, tuple((w3*self.render_scale).astype(int).tolist()), tuple((w4*self.render_scale).astype(int).tolist()), (0,0,0), 1)
        return img

if __name__ == "__main__":
    sim = SimulatorBicycle(render_scale=50.0)
    sim.init_pose((6, 6, 0))
    sim.render()
    cv2.imshow("Car", sim.render())
    cv2.waitKey(0)