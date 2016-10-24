from math import pi
half_pi = pi / 2
twopi = 2 * pi

def go_on(speed=1):
    def _go_on(self, believes: dict, actuators: dict, dt: float):
        self.set_velocity(speed)

    return _go_on


def turn(speed=1):
    def _turn(self, believes: dict, actuators: dict, dt: float):
        self.set_velocity(theta=speed)

    return _turn


def obstacle_avoidance(window=pi/2):
    def _obstacle_avoidance(self, believes: dict, actuators: dict, dt: float):
        closest_frontal = min(
            filter(lambda o: abs(o.theta) < window, self.get_obstacles()),
            key=lambda o: o.rho
        )

        if closest_frontal.theta >= 0:
            self.set_velocity(theta=closest_frontal.theta - half_pi)
        else:
            self.set_velocity(theta=half_pi + closest_frontal.theta)

    return _obstacle_avoidance
