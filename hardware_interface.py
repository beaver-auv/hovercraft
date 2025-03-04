import numpy as np

from hardware.motor_serial import MotorSerial
from hardware.sensors import VectorNavIMU, FakeDepth

class HovercraftHardware:
    def __init__(self, *, arduino_port, vectornav_port):
        self.motor_serial = MotorSerial(arduino_port) 
        self.imu = VectorNavIMU(vectornav_port)
        self.depth = FakeDepth()

        self.prev = {}

    def set_motor(self, pin, magnitude):
        if pin in self.prev and self.prev[pin] == magnitude:
            return
        self.prev[pin] = magnitude

        self.motor_serial.send({pin: magnitude})

    def initialize_motor(self, pin):
        pass

    def kill(self):
        self.motor_serial.kill()