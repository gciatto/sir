from abc import abstractmethod


class AbstractObstaclesExtractor:

    def __init__(self, belief_base: dict):
        self._believes = belief_base

    def extract_obstacles(self):
        self._believes['obstacles'] = \
            self._extract_obstacles(
                self._believes['points_of_interest'] if 'points_of_interest' in self._believes else [],
                self._believes['obstacles'] if 'obstacles' in self._believes else [],
            )

    @abstractmethod
    def _extract_obstacles(self, points_of_interest, obstacles):
        return []


class PoiAsObstacles(AbstractObstaclesExtractor):

    def _extract_obstacles(self, points_of_interest, obstacles):
        return points_of_interest


class CloserPoiFirst(AbstractObstaclesExtractor):

    def _extract_obstacles(self, points_of_interest, obstacles):
        return sorted(points_of_interest, key=lambda p: p[0])