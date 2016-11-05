from math import cos, sin, atan2, acos, sqrt, degrees, radians, pi

twopi = 2 * pi


def cartesian_to_polar(x, y, z, deg=False):
    rho = sqrt(sum((x * x, y * y, z * z)))
    if rho > 0:
        theta = atan2(y, x)
        phi = acos(z / rho)
    else:
        theta = phi = 0

    if deg:
        theta = degrees(theta)
        phi = degrees(phi)

    return rho, theta, phi


def polar_to_cartesian(rho, theta, phi, deg=False):
    if rho == 0:
        return 0, 0, 0
    else:
        if deg:
            theta = radians(theta)
            phi = radians(phi)

        x = rho * sin(phi) * cos(theta)
        y = rho * sin(phi) * sin(theta)
        z = rho * cos(phi)

        return x, y, z


def polar_to_polar(rho, theta, phi, f_angle):
    return rho, f_angle(theta), f_angle(phi)


def normalize_radians(a):
    a %= twopi
    if a > pi:
        return a - twopi
    elif a < -pi:
        return a + twopi
    else:
        return a


def normalize_polar_radians(v):
    rho, theta, phi = v
    return abs(rho), normalize_radians(theta), normalize_radians(phi)


def elementwise_sum(v1, v2):
    return tuple(map(sum, zip(v1, v2)))


def is_not_zero(v):
    return not all((x == 0 for x in v))


def arc_cos_sin(cosine, sine):
    angle = acos(cosine)
    if sine >= 0:
        return angle
    else:
        return -angle