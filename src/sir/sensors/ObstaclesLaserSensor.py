import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.sensors.laserscanner import LaserScanner
from morse.core import status
from morse.helpers.components import add_data, add_property


class Obstacleslasersensor(LaserScanner):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "Obstacleslasersensor"
    _short_desc = "Like LaserSensor but explicitly find obstacles"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('obstacles', 0.0, 'tuple', 'A dummy odometer, for testing purposes. Distance in meters')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super().__init__(self, obj, parent)

        # Do here sensor specific initializations

        self._distance = 0 # dummy internal variable, for testing purposes

        logger.info('Component initialized')

    def default_action(self):
        logger.info("%s default action" % self._name)

        super().default_action()

        self.local_data['obstacles'] = tuple((
            point for point in self.local_data['point_list'] if not all((coord == 0 for coord in point))
        ))

        logger.info("%s obstacles %s" % (self._name, self.local_data['obstacles']))

