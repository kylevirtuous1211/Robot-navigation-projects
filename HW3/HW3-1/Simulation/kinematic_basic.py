import sys

import numpy as np

sys.path.append("..")
from Simulation.kinematic import KinematicModel
from Simulation.utils import ControlState, State


class KinematicModelBasic(KinematicModel):
    def __init__(self, dt):
        # Simulation delta time
        self.dt = dt

    def step(self, state: State, cstate: ControlState) -> State:
        v = cstate.v
        w = cstate.w
        x = state.x + v * np.cos(np.deg2rad(state.yaw)) * self.dt
        y = state.y + v * np.sin(np.deg2rad(state.yaw)) * self.dt
        yaw = (state.yaw + state.w * self.dt) % 360
        state_next = State(x, y, yaw, v, w)
        return state_next
