#! /usr/bin/env python3
"""
Test client for the <sir> simulation environment.

This simple program shows how to control a robot from Python.

For real applications, you may want to rely on a full middleware,
like ROS (www.ros.org).
"""
import logging
import sys
from gciatto.utils import normalize_radians
from sir.client.controller import SirRobotController as Controller
from sir.client.strategies.behaviors import *
from sir.client.inspector.gui import InspectorGui
from sir.const import ROBOT_INITIAL_POSITION, ROBOT_INITIAL_ROTATION
from numpy import array, matrix
from math import radians
import sys

logger = logging.getLogger("morse.client")
logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.DEBUG)

gui = any(map(lambda x: x == "-gui", sys.argv))

try:
    from pymorse import Morse
except ImportError:
    logger.fatal("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

frequency = 5  # Hz
step_duration = 1 / frequency  # seconds


class SirInspectorGui(InspectorGui):
    def __init__(self, initial_pose=complex(0, 0), initial_bearing=.0):
        super().__init__(initial_pose, initial_bearing)

    def notify(self, sensors, believes, actuators, dt):
        data = dict()
        pose = sensors['pose']
        data['pose'] = complex(pose['x'], pose['y'])
        data['bearing'] = pose['yaw']
        if 'expected_state' in believes:
            data['state'] = believes['expected_state']
        if 'expected_state_covariances' in believes:
            data['covariances'] = believes['expected_state_covariances']
        super().notify(**data)


def simulate_odometry(now, prev, x, y, w, **kwargs):
    dt = now - prev
    odometry_data = dict()
    odometry_data['dx'] = x * dt
    odometry_data['dy'] = y * dt
    odometry_data['dz'] = 0
    odometry_data['dyaw'] = normalize_radians(w * dt)
    odometry_data['dpitch'] = 0
    odometry_data['droll'] = 0
    odometry_data['timestamp'] = now
    return odometry_data


try:
    inspector = None
    if gui:
        initial_pose = complex(*ROBOT_INITIAL_POSITION[0:2])
        initial_bearing = ROBOT_INITIAL_ROTATION[2]
        inspector = SirInspectorGui(initial_pose, initial_bearing)

    with Morse() as simulation:
        try:
            if gui:
                inspector.start()

            motion = simulation.robot.motion
            pose = simulation.robot.pose
            laser = simulation.robot.laser
            odometry = simulation.robot.odometry

            controller = Controller(
                name=simulation.robot.name + "_controller",
                logger=logger,
                inspector=inspector.notify if inspector is not None else None
            )

            controller.add_behavior(go_on(0.5)) \
                .add_behavior(obstacle_avoidance()) \
                .add_behavior(extended_kalman_filter())
            # controller.add_behavior(extended_kalman_filter())

            tend = 0
            no_move = dict(x=0, y=0, w=0)
            act = dict(motion=no_move)
            # odometry.subscribe(lambda o: controller.inject_sensors_data(odometry=o))

            while True:
                t0 = simulation.time()
                sensor_data = dict()
                sensor_data['laser'] = laser.get_local_data().result()
                t1 = simulation.time()
                # sensor_data['odometry'] = odometry.get_local_data().result()
                sensor_data['odometry'] = simulate_odometry(t1, tend, **act['motion'])
                t2 = simulation.time()
                sensor_data['pose'] = pose.get_local_data().result()
                t3 = simulation.time()
                act = controller.control_step(t3 - tend, **sensor_data)
                motion.publish(act['motion'])
                tend = simulation.time()
                elapsed = tend - t0
                if elapsed >= step_duration:
                    logger.warning("Step duration too high: %.2g s >= %.2g s" % (elapsed, step_duration))
                    pass
                else:
                    simulation.sleep(step_duration - elapsed)
        except KeyboardInterrupt as k:
            motion.publish(no_move)
except Exception as e:
    logger.exception(e)
