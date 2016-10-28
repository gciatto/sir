from math import pi

def zoom(x):
    return 2 ** (x / 10)


U = 50


def draw_axes(canvas, cc, axis_units=10):
    cc.move_to(-U, 0)
    cc.line_to(axis_units * U, 0)
    cc.move_to(0, -U)
    cc.line_to(0, axis_units * U)
    cc.stroke()


Or = 5


def draw_active(canvas, cc, radius_units=1):
    ar = radius_units * Or
    cc.arc(0, 0, ar, 0, 2 * pi)
    cc.stroke()
    cc.move_to(0, 0)
    cc.line_to(2 * ar, 0)
    cc.stroke()


def draw_passive(canvas, cc, radius_units=1):
    ar = radius_units * Or
    cc.rectangle(-ar, -ar, 2 * ar, 2 * ar)
    cc.stroke()
