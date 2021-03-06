#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <sir> environment

Feel free to edit this template as you like!
"""
# from morse.robots.quadrotor import
from morse.builder import *
from sir.const import *

robot = ATRV()

laser = Hokuyo()
laser.properties(
    laser_range=LASER_RANGE,
    scan_window=LASER_SCAN_WINDOW,
    resolution=LASER_RESOLUTION,
    Visible_arc=LASER_ARC_VISIBLE,
    frequency=LASER_FREQUENCY
)
laser.alter('', 'sir.modifiers.LaserZeroMeanGaussianNoiseModifier')
laser.translate(0, 0, 0.715)
robot.append(laser)

odometry = Odometry()
odometry.properties(frequency=ODOMETRY_FREQUENCY)
odometry.level('differential')
# odometry.alter('', 'sir.modifiers.OdometryZeroMeanGaussianNoiseModifier')
odometry.translate(0, 0, 0.5)
robot.append(odometry)

motion = MotionXYW()
robot.append(motion)

keyboard = Keyboard()
robot.append(keyboard)
keyboard.properties(ControlType='Position')

pose = Pose()
# pose.properties(frequency=1)
robot.append(pose)

# {'roll': -1.0039110520665417e-07, 'yaw': -0.00026614448870532215, 'timestamp': 1477327740.063312, 'x': -7.427310466766357, 'pitch': 9.00480685572802e-08, 'y': 7.127953052520752, 'z': 0.059999980032444}
robot.translate(*ROBOT_INITIAL_POSITION)
robot.rotate(*ROBOT_INITIAL_ROTATION)

robot.add_default_interface('socket')

env = Environment('./res/arenas/boxes_nowalls_atall.blend', fastmode=False)
env.set_time_strategy(TimeStrategies.FixedSimulationStep)
env.show_framerate(True)
# env.simulator_frequency(1/10)
# env = Environment('tum_kitchen/tum_kitchen', fastmode=True)
env.set_camera_location([0, 0, 40])
env.set_camera_rotation([0, 0, 0])
