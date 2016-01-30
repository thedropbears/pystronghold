from robot_map import RobotMap

from wpilib.counter import Counter

class RangeFinder():

    def __init__(self):
        self.counter = Counter(RobotMap.range_finder_dio_channel)
        self.counter.setSemiPeriodMode(highSemiPeriod=True)

    def getDistance(self):
        return self.counter.getPeriod()*1000000/10 # 10 usec is 1cm, return as cm
