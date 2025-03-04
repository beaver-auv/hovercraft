import numpy as np
import quaternion
from vnpy import VnSensor
from ezauv.hardware.sensor_interface import ImuInterface, DepthInterface


class VectorNavIMU(ImuInterface):
    def __init__(self, port):
        self.vectornav = VnSensor()
        self.vectornav.connect(port, 115200)
        self.calibrated_heading = 0

    def get_accelerations(self) -> np.ndarray:
        accel = self.vectornav.read_yaw_pitch_roll_magnetic_acceleration_and_angular_rates().accel
        return np.array([accel.x, accel.y, accel.z])

    def get_rotation(self) -> np.quaternion:
        rot = self.vectornav.read_yaw_pitch_roll()
        return quaternion.from_euler_angles(rot.x, rot.y, rot.z)

    def initialize(self) -> None:
        interval = 0.01
        total = 5 / interval
        self.log(f"Calibrating IMU heading over 5 seconds ({total} checks)...")
        heading_sum = 0
        for i in np.linspace(0, 5, interval):
            heading_sum += self.vectornav.read_yaw_pitch_roll().x
        self.calibrated_heading = heading_sum / total

    def overview(self) -> None:
        self.log(f"VectorNav IMU --- {self.vectornav.read_model_number()}")

class FakeDepth(DepthInterface):
    # this sucks ass but the library needs depth sensor rn for some reason

    def __init__(self):
        super().__init__()
        self.log = lambda str: print(str)
    
    def get_depth(self) -> float:
        return 0.
    
    def initialize(self) -> None:
        pass

    def overview(self) -> None:
        self.log("remember to fix this later")