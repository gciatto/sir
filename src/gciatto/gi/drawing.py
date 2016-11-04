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


Or = 15


def fill_arc(canvas, cc, half_angle, radius_units=1):
    ar = radius_units * Or
    cc.arc(0, 0, ar, -half_angle, half_angle)
    cc.fill()


def fill_ellipse(canvas, cc, h_radius, v_radius, angle=0, radius_units=1):
    if h_radius > 0 or v_radius > 0:

        cc.save()
        cc.scale(h_radius * radius_units, v_radius * radius_units)
        cc.rotate(angle)
        cc.arc(0, 0, 1, 0, 2 * pi)
        cc.fill()
        cc.restore()


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
