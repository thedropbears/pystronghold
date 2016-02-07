import wpilib

class RangeFinder:

    range_finder_counter = wpilib.Counter

    def getDistance(self):
        return self.range_finder_counter.getPeriod() * 1000000 / 1000  # 10 usec is 1cm, return as metres

    def execute(self):
        pass
