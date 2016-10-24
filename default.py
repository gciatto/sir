#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <sir> environment

Feel free to edit this template as you like!
"""
# from morse.robots.quadrotor import
from sir.builder.sensors import *
from sir.const.laser import *
from morse.builder import *

robot = B21()

laser = Hokuyo()
laser.properties(
    laser_range=RANGE,
    scan_window=SCAN_WINDOW,
    resolution=RESOLUTION,
    Visible_arc=ARC_VISIBLE,
    frequency=FREQUENCY
)
laser.alter('', 'sir.modifiers.LaserZeroMeanGaussianNoiseModifier')
laser.translate(0, 0, 0.715)
robot.append(laser)

odometry = Odometry()
odometry.level('differential')
odometry.alter('', 'sir.modifiers.OdometryZeroMeanGaussianNoiseModifier')
odometry.translate(0, 0, 0.96)
robot.append(odometry)

motion = MotionXYW()
robot.append(motion)

keyboard = Keyboard()
robot.append(keyboard)
keyboard.properties(ControlType='Position')

pose = Pose()
robot.append(pose)

# {'roll': -1.0039110520665417e-07, 'yaw': -0.00026614448870532215, 'timestamp': 1477327740.063312, 'x': -7.427310466766357, 'pitch': 9.00480685572802e-08, 'y': 7.127953052520752, 'z': 0.059999980032444}
robot.translate(-7.45, 7.10, 0.06)
robot.rotate(0.0, 0.0, 180)

robot.add_default_interface('socket')

env = Environment('indoors-1/indoor-1', fastmode=True)
# env = Environment('tum_kitchen/tum_kitchen', fastmode=True)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
