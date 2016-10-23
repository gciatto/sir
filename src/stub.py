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

logger = logging.getLogger("morse.client")
logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.DEBUG)

try:
    from pymorse import Morse
except ImportError:
    logger.fatal("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

frequency = 2  # Hz
step_duration = 1 / frequency  # seconds

try:
    with Morse() as simulation:
        motion = simulation.robot.motion
        pose = simulation.robot.pose
        laser = simulation.robot.laser
        odometry = simulation.robot.odometry

        controller = Controller(simulation.robot.name + "_controller", logger)

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
except Exception as e:
    logger.exception(e)