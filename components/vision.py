from networktables import NetworkTable
from wpilib.interfaces import PIDSource
from vision.vision import setCaptureParameters
import inspect
import hal
import os

class Vision:
    def __init__(self):
        # mjpg-streamer isn't setting parameters properly yet, so do it here
        if not hal.HALIsSimulation():
            setCaptureParameters("/dev/v4l/by-id/usb-046d_0825_96EBCE50-video-index0",
                                 "/etc/default/mjpg-streamer")
        self.nt = NetworkTable.getTable('vision')
        self._values = {'x': 0.0, 'y': 0.0, 'w': 0.0, 'h': 0.0, 'time': 0.0}
        self._smoothed_pidget = 0.0
        self.no_vision_counter = 0
        self.nt.addTableListener(self.valueChanged)

    def valueChanged(self, table, key, value, isNew):
        self._values[key] = float(value)
        if key == 'time':
            # The time key is updated last, so let's update our smoothed average
            alpha = 0.3
            if self._values['w'] > 0.0:
                self._smoothed_pidget = (alpha * self._values['x']
                                         + (1.0 - alpha) * self._smoothed_pidget)
                self.no_vision_counter = 0
            else:
                self.no_vision_counter += 1

    def getPIDSourceType(self):
        return PIDSource.PIDSourceType.kDisplacement

    def pidGet(self):
        return -self._smoothed_pidget
