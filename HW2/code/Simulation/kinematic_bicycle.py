import numpy as np
import sys
sys.path.append("..")
from Simulation.utils import State, ControlState
from Simulation.kinematic import KinematicModel

class KinematicModelBicycle(KinematicModel):
    def __init__(self,
            l = 30,     # distance between rear and front wheel
            dt = 0.05
        ):
        # Distance from center to wheel
        self.l = l
        # Simulation delta time
        self.dt = dt

    def step(self, state:State, cstate:ControlState) -> State:
        # TODO 2.3.1: Bicycle Kinematic Model
        a = cstate.a           # linear acceleration (deg/s² based on simulator)
        delta = cstate.delta   # front-wheel steering angle (degrees)
        x, y, yaw = state.x, state.y, state.yaw

        # Integrate velocity from acceleration
        v = state.v + np.deg2rad(a) * self.dt

        # Yaw from Ackermann steering geometry
        w = v * np.tan(np.deg2rad(delta)) / self.l

        # Kinematic update (rear-axle reference point)
        x = x + v * np.cos(np.deg2rad(yaw)) * self.dt
        y = y + v * np.sin(np.deg2rad(yaw)) * self.dt
        yaw = yaw + np.rad2deg(w) * self.dt
        # [end] TODO 2.3.1
        state_next = State(x, y, yaw, v, w)
        return state_next
