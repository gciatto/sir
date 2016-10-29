#! /usr/bin/env python3
"""
Test client for the <sir> simulation environment.

This simple program shows how to control a robot from Python.

For real applications, you may want to rely on a full middleware,
like ROS (www.ros.org).
"""
import logging
import sys
from sir.client.controller import SirRobotController as Controller
from sir.client.strategies.behaviors import *
from sir.client.inspector.gui import InspectorGui
from sir.const import ROBOT_INITIAL_POSITION, ROBOT_INITIAL_ROTATION
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

frequency = 2  # Hz
step_duration = 1 / frequency  # seconds


class SirInspectorGui(InspectorGui):
    def __init__(self, initial_pose=complex(0, 0), initial_bearing=.0):
        super().__init__(initial_pose, initial_bearing)

    def notify(self, sensors, believes, actuators, dt):
        pose = sensors['pose']
        super().notify(
            pose=complex(pose['x'], pose['y']),
            bearing=pose['yaw']
        )


try:
    inspector = None
    if gui:
        initial_pose = complex(*ROBOT_INITIAL_POSITION[0:2])
        initial_bearing = radians(ROBOT_INITIAL_ROTATION[2])
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

            controller.add_behavior(go_on()) \
                .add_behavior(obstacle_avoidance())

            tprev = tend = 0

            while True:
                sensor_data = dict()
                tprev = tend
                t0 = simulation.time()
                sensor_data['laser'] = laser.get_local_data().result()
                t1 = simulation.time()
                sensor_data['odometry'] = odometry.get_local_data().result()
                t2 = simulation.time()
                sensor_data['pose'] = pose.get_local_data().result()
                t3 = simulation.time()
                act = controller.control_step(tend - tprev, **sensor_data)
                motion.publish(act['motion'])
                t4 = simulation.time()
                elapsed = t4 - t0
                if elapsed >= step_duration:
                    logger.warning("Step duration too high: %.2g s >= %.2g s" % (elapsed, step_duration))
                    pass
                else:
                    simulation.sleep(step_duration - elapsed)
                tend = simulation.time()
        except KeyboardInterrupt as k:
            motion.publish(dict(x=0, y=0, w=0))
except Exception as e:
    logger.exception(e)
