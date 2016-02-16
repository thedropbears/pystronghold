import wpilib

import math

class RangeFinder:

    range_finder_counter = wpilib.Counter

    angle_inclination = 10.0 # deg

    def getDistance(self):
        return self.range_finder_counter.getPeriod() * 1000000 / 1000 * math.tan(math.degrees(self.angle_inclination))  # 10 usec is 1cm, return as metres

    def pidGet(self):
        return self.getDistance()

    def execute(self):
        pass
