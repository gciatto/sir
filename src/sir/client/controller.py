from abc import abstractmethod
import logging
from gciatto.utils import *
from operator import itemgetter
from collections import namedtuple
from itertools import chain
from sir.client.strategies.obstacles import PoiAsObstacles
from types import MethodType
from functools import partialmethod

logger = logging.getLogger()


class AbstractRobotController:
    """
    Rappresenta il SW di controllo di un robot governato da uno stub di MORSE o, in senso lato, il robot stesso.

    Ogni robot è identificatio da un nome, fornito al momento dell'inizializzazione.
    Lo stub deve iniettare i dati sensoriali forniti da MORSE, dopo di che è compito del controllore creare delle
    believes a partire dai essi. Le believes sono usate per impartire i comandi agli attuatori.
    """

    _used_names = set()

    def __init__(self, name, logger=logger, inspector=None):
        if name in AbstractRobotController._used_names:
            raise Exception("Name `%s` already used" % name)
        else:
            AbstractRobotController._used_names.add(name)
        self._name = name
        self._sensors = dict()
        self._believes = dict()
        self._actuators = dict()
        self._inspect = inspector
        self.logger = logger.getChild("controllers." + name)

    def __del__(self):
        AbstractRobotController._used_names.remove(self._name)

    @property
    def name(self):
        return self._name

    def inject_sensors_data(self, **sensors_data):
        """
        Permette a un entità esterna di iniettare i dati sensoriali, eventualmente sostituendo quelli precedenti
        :param sensors_data: coppie chiave-valore dove la chiave rappresenta il sensore e il valore il suo nuovo dato

        :return: None
        """

        self._sensors.update(**sensors_data)

    @abstractmethod
    def _update_believes(self, sensors: dict, believes: dict, dt: float):
        """
        Provoca dei side effect nel campo `believes` al fine di modificare le attuali convinzioni del robot.

        :param sensors: i dati sensoriali più recenti, NON modificabili
        :param believes: le convinzioni conrrenti, modificabili
        :param dt: il tempo trascorso dall'ultimo aggiornamento delle convinzioni
        :return: None
        """

        pass

    @abstractmethod
    def _update_actuators(self, believes: dict, actuators: dict, dt: float):
        """
        Aggiorna gli attuatori.

        :param believes: le convinzioni conrrenti, NON modificabili
        :param actuators: i dati da inviare agli attuatori, modificabili
        :param dt: il tempo trascorso dall'ultimo aggiornamento degli attuatori
        :return: None
        """

        pass

    def control_step(self, dt, **sensors_data):
        """
        Permette a un'entità esterna di eseguire uno step del loop di controllo.

        :param dt: il tempo trascorso dall'esecuzione dell'ultimo step
        :param sensors_data: i dati sensorali da iniettare
        :return: i dati aggiornati da inviare agli attuatori
        """
        self.inject_sensors_data(**sensors_data)
        self._update_believes(self._sensors, self._believes, dt)
        self._update_actuators(self._believes, self._actuators, dt)
        if self._inspect is not None:
            self._inspect(self._sensors, self._believes, self._actuators, dt)
        return self._actuators

    def add_behavior(self, behavior):
        old_layer = self._update_actuators
        new_layer = MethodType(behavior, self)

        def _method(self, *args, **kwargs):
            old_layer(*args, **kwargs)
            new_layer(*args, **kwargs)

        self._update_actuators = MethodType(_method, self)
        return self


PointOfInterest = namedtuple('PointOfInterest', ['rho', 'theta', 'phi', 'x', 'y', 'z'])
Speed = namedtuple('Speed', ['x', 'y', 'theta'])
Variation = namedtuple('Variation', ['dx', 'dy', 'dtheta'])


def _cartesian_to_poi(p):
    polar = cartesian_to_polar(*p)
    normalized = polar  # normalize_polar_radians(polar)
    return PointOfInterest(*chain(normalized, p))


class SirRobotController(AbstractRobotController):
    def __init__(self, name, obstacles_strategy=PoiAsObstacles, *args, **kwargs):
        super().__init__(name, *args, **kwargs)
        self._obstacles_extractor = obstacles_strategy(self._believes)
        self._actuators['motion'] = dict(x=0, y=0, w=0)

    def _update_believes(self, sensors: dict, believes: dict, dt: float):
        believes['points_of_interest'] = tuple(
            map(
                _cartesian_to_poi,
                filter(
                    is_not_zero,
                    self.get_laser_points()
                )
            )
        )
        # self.logger.debug("Points of interest: %s", self.get_points_of_interest())

        self._extract_obstacles()
        # self.logger.debug("Obstacles: %s", self.get_obstacles())

        if len(self.get_obstacles()) > 0:
            closest = min(self.get_obstacles(), key=lambda x: x[0])
            self.logger.debug("Closest Obstacle: (%s, %s, %s)" % polar_to_polar(*closest[0:3], f_angle=degrees))

        if 'odometry' in sensors:
            odometry = sensors['odometry']
            believes['odometry'] = odometry
            self.logger.debug("Odometry: (dx, dy, dtheta) = (%s, %s, %s)" % (odometry['dx'], odometry['dy'], odometry['dyaw']))

    def get_odometry(self):
        data = self._believes['odometry']
        if data is None:
            return Variation(0, 0, 0)
        else:
            return Variation(data['dx'], data['dy'], data['dyaw'])

    def _extract_obstacles(self):
        self._obstacles_extractor.extract_obstacles()

    def _update_actuators(self, believes: dict, actuators: dict, dt: float):
        pass  # self.logger.debug("_update_actuators")

    def get_velocity(self):
        m = self._actuators['motion']
        return Speed(m['x'], m['y'], m['w'])

    def set_velocity(self, x=None, theta=None, y=None):
        if 'motion' not in self._actuators:
            m = dict(x=0, y=0, w=0)
            self._actuators['motion'] = m
        else:
            m = self._actuators['motion']
        if x:
            m['x'] = x
        if y:
            m['y'] = y
        if theta:
            m['w'] = theta

    def inc_velocity(self, x=0, theta=0, y=0):
        if 'motion' not in self._actuators:
            m = dict(x=0, y=0, w=0)
            self._actuators['motion'] = m
        else:
            m = self._actuators['motion']
        m['x'] += x
        m['y'] += y
        m['w'] += theta

    def get_laser_points(self):
        return self._sensors['laser']['point_list']

    def get_points_of_interest(self):
        return self._believes['points_of_interest']

    def get_obstacles(self):
        return self._believes['obstacles']



