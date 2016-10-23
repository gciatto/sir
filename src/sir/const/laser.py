from math import radians

RANGE = 5  # meters
SCAN_WINDOW = 360  # degrees
RESOLUTION = 5  # degrees
FREQUENCY = 5  # Hz

ARC_VISIBLE = True  # shows the perceived arc within simulation

RANGE_STDEV = 0.0025  # 2.5 cm
HORIZONTAL_ANGLE_STDEV = radians(RESOLUTION / 2)  # 2°
VERTICAL_ANGLE_STDEV = radians(1)  # 1°



