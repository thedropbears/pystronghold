
from components.chassis import Chassis

class LowBarTower:
    LEFT = 1
    CENTRE = 2
    RIGHT = 3

    chassis = Chassis

    def on_enable(self):
        # Reset the IMU
        self.chassis.bno055.resetHeading()
        self.chassis.heading_hold = True

    def on_disable(self):
        pass

    def on_iteration(self, tm):
        # First we run on odometry
        pass


class LowBarCentreTower(LowBarTower):
    MODE_NAME = "Low bar, CENTRE tower"
    DEFAULT = True

    target = LowBarTower.CENTRE


class LowBarLeftTower(LowBarTower):
    MODE_NAME = "Low bar, LEFT tower"
    DEFAULT = False

    target = LowBarTower.LEFT


class LowBarRightTower(LowBarTower):
    MODE_NAME = "Low bar, RIGHT tower"
    DEFAULT = False

    target = LowBarTower.RIGHT

