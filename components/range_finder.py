import wpilib

import math

from wpilib.interfaces import PIDSource

class RangeFinder(PIDSource):

    def __init__(self, dio_number):
        self.range_finder_counter = wpilib.Counter(dio_number)
        self.range_finder_counter.setSemiPeriodMode(highSemiPeriod=True)
        self._smoothed_d = 0.0

    def getDistance(self):
        return self.range_finder_counter.getPeriod() * 1000000 / 1000 # 10 usec is 1cm, return as metres

    def getPIDSourceType(self):
        return PIDSource.PIDSourceType.kDisplacement

    def pidGet(self):
        alpha = 0.7
        d = self.getDistance()
        self._smoothed_d = alpha * d + (1.0 - alpha) * self._smoothed_d
        return self._smoothed_d
