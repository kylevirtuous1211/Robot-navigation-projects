import sys
import numpy as np
import cv2

sys.path.append("..")
from Simulation.simulator import Simulator
import Simulation.utils as utils
from Simulation.utils import State, ControlState
from Simulation.kinematic_basic import KinematicModelBasic as KinematicModel

#wheeled mobile robotics
class SimulatorBasic(Simulator):
    def __init__(self,
            v_range = 90.0,
            w_range = 180.0,
            a_range = (-20.0, 15.0),
            l = 0.5,     # distance from center to wheel
            wu = 0.3,    # wheel length
            wv = 0.1,    # wheel width
            car_w = 0.8, # car width
            car_f = 0.8, # car front bumper length
            car_r = 0.8, # car rear bumper length
            dt = 0.05,
            render_scale = 10.0 # Multiplier to convert meters to pixels for drawing
        ):
        self.control_type = "basic"
        # Control Constrain
        self.v_range = v_range
        self.w_range = w_range
        self.a_range = a_range
        
        # Track previous velocity to calculate acceleration
        self.p_v = 0.0
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
        self.model = KinematicModel(dt)

        # Initialize State
        self.state = State()
        self.cstate = ControlState(self.control_type, 0.0, 0.0)
        self.car_box = utils.compute_car_box(self.car_w, self.car_f, self.car_r, self.state.pose())
    
    def init_pose(self, pose):
        self.state.update(pose[0], pose[1], pose[2])
        self.cstate = ControlState(self.control_type, 0.0, 0.0)
        self.p_v = 0.0
        self.car_box = utils.compute_car_box(self.car_w, self.car_f, self.car_r, self.state.pose())
        self.record = []
        return self.state, {}

    def step(self, command, update_state=True):
        if command is not None:
            # Check Control Command
            self.cstate.v = command.v if command.v is not None else self.cstate.v 
            self.cstate.w = command.w if command.w is not None else self.cstate.w

        # Linear Acceleration Constrain (Rate of Change)
        v_diff_limit_max = self.a_range[1] * self.dt
        v_diff_limit_min = self.a_range[0] * self.dt
        
        if self.cstate.v - self.p_v > v_diff_limit_max:
            self.cstate.v = self.p_v + v_diff_limit_max
        elif self.cstate.v - self.p_v < v_diff_limit_min:
            self.cstate.v = self.p_v + v_diff_limit_min
            
        self.p_v = self.cstate.v

        # Linear Velocity Constrain
        if self.cstate.v > self.v_range:
            self.cstate.v = self.v_range
        elif self.cstate.v < -self.v_range:
            self.cstate.v = -self.v_range
        if self.cstate.w > self.w_range:
            self.cstate.w = self.w_range
        elif self.cstate.w < -self.w_range:
            self.cstate.w = -self.w_range

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
        # 1 Wheel
        wu_sc, wv_sc = int(np.clip(self.wu*self.render_scale, 1, a_max=None)), int(np.clip(self.wv*self.render_scale, 1, a_max=None))
        img = utils.draw_rectangle(img,int(self.state.x*self.render_scale),int(self.state.y*self.render_scale),wu_sc,wv_sc,-self.state.yaw)
        # img = utils.draw_rectangle(img,int(w1[0]*self.render_scale),int(w1[1]*self.render_scale),wu_sc,wv_sc,-self.state.yaw)
        # img = utils.draw_rectangle(img,int(w2[0]*self.render_scale),int(w2[1]*self.render_scale),wu_sc,wv_sc,-self.state.yaw)
        # Axle
        img = cv2.line(img, tuple((w1*self.render_scale).astype(int).tolist()), tuple((w2*self.render_scale).astype(int).tolist()), (0,0,0), 1)
        return img

if __name__ == "__main__":
    sim = SimulatorBasic(render_scale=100.0)
    sim.init_pose((3, 3, 0))
    sim.render()
    cv2.imshow("Car", sim.render())
    cv2.waitKey(0)