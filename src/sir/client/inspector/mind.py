from abc import abstractmethod


class RobotObserver:
    def __init__(self, x=0, y=0, theta=0):
        super().__init__()
        self.pose = complex(x, y)
        self.bearing = theta

    def _event(self):
        return dict(
            pose=self.pose,
            bearing=self.bearing
        )

    def _notify(self):
        self.notify(**self._event())

    @abstractmethod
    def notify(self, **kwargs):
        pass

