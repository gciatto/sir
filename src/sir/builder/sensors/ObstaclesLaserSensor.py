from morse.builder.sensors import LaserSensorWithArc


class Obstacleslasersensor(LaserSensorWithArc):
    _classpath = "sir.sensors.ObstaclesLaserSensor.Obstacleslasersensor"
    _blendname = "obstacleslasersensor"
    _name = "ObstaclesLaserSensor"
    _short_desc = "Laser scanner which explicitly select obstacles"

    def __init__(self, name=None):
        super().__init__(self, name, Obstacleslasersensor._classpath)

