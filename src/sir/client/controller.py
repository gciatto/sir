from abc import abstractmethod
import logging

logger = logging.getLogger()

class AbstractRobotController:
    """
    Rappresenta il SW di controllo di un robot governato da uno stub di MORSE o, in senso lato, il robot stesso.

    Ogni robot è identificatio da un nome, fornito al momento dell'inizializzazione.
    Lo stub deve iniettare i dati sensoriali forniti da MORSE, dopo di che è compito del controllore creare delle
    believes a partire dai essi. Le believes sono usate per impartire i comandi agli attuatori.
    """

    _used_names = set()

    def __init__(self, name, logger=logger):
        if name in AbstractRobotController._used_names:
            raise Exception("Name `%s` already used" % name)
        else:
            AbstractRobotController._used_names.add(name)
        self._name = name
        self._sensors = dict()
        self._believes = dict()
        self._actuators = dict()
        self.logger = logger.getChild("controllers." + name)

    def __del__(self):
        AbstractRobotController._used_names.remove(self._name)

    @property
    def name(self):
        return self._name

    def _inject_sensors_data(self, **sensors_data):
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
        self._inject_sensors_data(**sensors_data)
        self._update_believes(self._sensors, self._believes, dt)
        self._update_actuators(self._believes, self._actuators, dt)
        return self._actuators


class SirRobotController(AbstractRobotController):
    def __init__(self, name, *args, **kwargs):
        super().__init__(name, *args, **kwargs)

    def _update_believes(self, sensors: dict, believes: dict, dt: float):
        self.logger.debug(repr(sensors))

    def _update_actuators(self, believes: dict, actuators: dict, dt: float):
        self.set_velocity(0.5)

    def set_velocity(self, forward=0, angular=0, sideward=0):
        self._actuators.update(motion=dict(x=forward, y=sideward, w=angular))