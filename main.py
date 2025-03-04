import numpy as np
import quaternion

from ezauv.auv import AUV
from ezauv.hardware import MotorController, Motor, SensorInterface
from ezauv.utils.inertia import InertiaBuilder, Cuboid
from ezauv.mission.tasks.main import AccelerateVector
from ezauv.mission.tasks.subtasks import HeadingPID
from ezauv.mission import Path

from hardware_interface import HovercraftHardware


motor_locations = [
    np.array([-1., -1., 0.]),
    np.array([-1., 1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.])
    ]

motor_directions = [
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.])
    ]



bounds = [[-0.2, 0.2]] * 4
deadzone = [[-0.11, 0.11]] * 4

hardware = HovercraftHardware(
    arduino_port='/dev/ttyUSB1',
    vectornav_port='/dev/ttyUSB0'
)

anchovy = AUV(
    motor_controller = MotorController(
        inertia = InertiaBuilder(
            Cuboid(
                mass=1,
                width=1,
                height=1,
                depth=0.1,
                center=np.array([0,0,0])
            )).moment_of_inertia(),
            motors = [
                Motor(
                    direction,
                    loc,
                    lambda magnitude: hardware.set_motor(i, magnitude),
                    lambda: hardware.initialize_motor(i),
                    Motor.Range(bounds[i][0], bounds[i][1]),
                    Motor.Range(-deadzone[i][0], deadzone[i][1])
                    )
                for i, (loc, direction) in enumerate(zip(motor_locations, motor_directions))
                ]
        ),
        sensors = SensorInterface(imu=hardware.imu(), depth=hardware.depth()),
        pin_kill = lambda: None
    )

anchovy.register_subtask(HeadingPID(0, 0.03, 0.0, 0.01))


mission = Path(
    AccelerateVector(np.array([0., 0., 0., 0., 0., 0.]), 20)
)

anchovy.travel_path(mission)