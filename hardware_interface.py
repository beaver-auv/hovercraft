import numpy as np

from hardware.motor_serial import MotorSerial
from hardware.sensors import VectorNavIMU, FakeDepth

class HovercraftHardware:
    def __init__(self, *, arduino_port, vectornav_port):
        self.motor_serial = MotorSerial(arduino_port) 
        self.imu = VectorNavIMU(vectornav_port)
        self.depth = FakeDepth()

    def set_motor(self, pin, magnitude):
        self.motor_serial.send({pin: magnitude})

    def initialize_motor(self, pin):
        pass