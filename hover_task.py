from ezauv.mission.mission import Subtask
from ezauv.hardware.sensor_interface import SensorInterface
from ezauv.utils.pid import PID

import numpy as np


class Hover(Subtask):

    def __init__(self, wanted_acceleration):
        super().__init__()
        self.vertical_acceleration = wanted_acceleration
        
    @property
    def name(self) -> str:
        return "Hover subtask"

    def update(self, sensors: SensorInterface, wanted_speed: np.ndarray) -> np.ndarray:
        return np.array([0., 0., self.vertical_acceleration, 0., 0., 0.])