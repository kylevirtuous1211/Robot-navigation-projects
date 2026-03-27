import abc
import sys

sys.path.append("..")
from Simulation.utils import ControlState, State


class KinematicModel:
    @abc.abstractmethod
    def step(self, state: State, cstate: ControlState) -> State:
        return NotImplementedError
