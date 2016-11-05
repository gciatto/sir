import logging
from random import gauss

from morse.modifiers.abstract_modifier import AbstractModifier

from gciatto.utils import *
from sir.const import LASER_RANGE_STDEV, LASER_HORIZONTAL_ANGLE_STDEV, LASER_VERTICAL_ANGLE_STDEV, ODOMETRY_DX_STDEV, \
    ODOMETRY_DY_STDEV, ODOMETRY_DZ_STDEV, ODOMETRY_DYAW_STDEV, ODOMETRY_DROLL_STDEV, ODOMETRY_DPITCH_STDEV

_L = logging.getLogger("morse." + __name__)


class OdometryZeroMeanGaussianNoiseModifier(AbstractModifier):

    def initialize(self):
        self._dx_stdev = float(self.parameter("dx_stdev", default=ODOMETRY_DX_STDEV))
        self._dy_stdev = float(self.parameter("dy_stdev", default=ODOMETRY_DY_STDEV))
        self._dz_stdev = float(self.parameter("dz_stdev", default=ODOMETRY_DZ_STDEV))

        self._dpitch_stdev = float(self.parameter("dpitch_stdev", default=ODOMETRY_DPITCH_STDEV))
        self._droll_stdev = float(self.parameter("droll_stdev", default=ODOMETRY_DROLL_STDEV))
        self._dyaw_stdev = float(self.parameter("dyaw_stdev", default=ODOMETRY_DYAW_STDEV))

    def modify(self):
        try:
            dx, dy, dz = \
                gauss(0, self._dx_stdev), \
                gauss(0, self._dy_stdev), \
                gauss(0, self._dz_stdev)

            dyaw, droll, dpitch = \
                gauss(0, self._dyaw_stdev), \
                gauss(0, self._droll_stdev), \
                gauss(0, self._dpitch_stdev)

            self.data['dx'] += dx
            self.data['dy'] += dy
            self.data['dz'] += dz
            self.data['dyaw'] += dyaw
            self.data['dpitch'] += dpitch
            self.data['droll'] += droll

            _L.info("%s Noise: (dx, dy, dtheta) += (%.1e, %.1e, %.1e) --> (%.1e, %.1e, %.1e) " % (self.component_name, dx, dy, dyaw, self.data['dx'], self.data['dy'], self.data['dyaw']))
        except KeyError as detail:
            self.key_error(detail)


class LaserZeroMeanGaussianNoiseModifier(AbstractModifier):

    # _LaserDatum = namedtuple("LaserDatum", ('rho', 'theta', 'phi'))

    def initialize(self):
        self._drho_stdev = float(self.parameter("drho_stdev", default=LASER_RANGE_STDEV))
        self._dtheta_stdev = float(self.parameter("dtheta_stdev", default=LASER_HORIZONTAL_ANGLE_STDEV))
        self._dphi_stdev = float(self.parameter("dphi_stdev", default=LASER_VERTICAL_ANGLE_STDEV))

    def modify(self):

        def _noise():
            while True:
                drho, dtheta, dphi = \
                    gauss(0, self._drho_stdev), \
                    gauss(0, self._dtheta_stdev), \
                    gauss(0, self._dphi_stdev)

                yield (drho, dtheta, dphi)

        try:
            points_cart = self.data['point_list']  # filter(is_not_zero, self.data['point_list'])
            points_polar = (cartesian_to_polar(*v) for v in points_cart)
            noisy_polars = (
                elementwise_sum(*vs) if vs[0][0] > 0 else vs[0]
                for vs in zip(points_polar, _noise())
            )
            normalized = list(map(normalize_polar_radians, noisy_polars))

            self.data['range_list'][:] = list((v[0] for v in normalized))
            self.data['point_list'][:] = list((polar_to_cartesian(*v) for v in normalized))

            # printable = sorted(
            #     [polar_to_polar(*v, f_angle=degrees) for v in normalized if is_not_zero(v)],
            #     key=lambda p: p[0]
            # )
            #
            # _L.info("%s Obstacles: %s" % (self.component_name, printable))
        except KeyError as detail:
            self.key_error(detail)