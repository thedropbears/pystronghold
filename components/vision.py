import multiprocessing
import multiprocessing.sharedctypes
import os
import time
import cv2
import numpy as np
from wpilib import Resource
import hal

import logging
import argparse

from wpilib.interfaces import PIDSource

class Vision:
    video_width = 320
    video_height = 240
    video_contrast = 0.5
    video_brightness = 0.5
    video_saturation = 0.5
    video_exposure = 5  # min 3 max 2047
    video_white_balance = 4000  # min 2000 max 6500
    video_gain = 0.0
    def __init__(self):
        self._data_array = multiprocessing.sharedctypes.RawArray("d", [0.0, 0.0, 0.0, 0.0, 0.0])
        self._process_run_event = multiprocessing.Event()
        self.logger = logging.getLogger("vision")
        self._vision_process = VisionProcess(self._data_array, self._process_run_event)
        self._process_run_event.set()
        self._vision_process.daemon = True
        self._vision_process.start()
        self.logger.info("Vision process started: ")
        # Register with Resource so teardown works
        Resource._add_global_resource(self)

    def free(self):
        self._process_run_event.clear()
        self._vision_process.join(0.1)
        self._vision_process.terminate()

    def get(self):
        if self._data_array[2] > 0.0:
            # New value and we have a target
            return self._data_array[0:4]
        else:
            return None

    def getPIDSourceType(self):
        return PIDSource.PIDSourceType.kDisplacement

    def pidGet(self):
        return -self._data_array[0]

    def execute(self):
        pass


class VisionProcess(multiprocessing.Process):
    def __init__(self, data_array, run_event):
        super().__init__(args=(data_array, run_event))
        self.vision_data_array = data_array
        self._run_event = run_event
        self.logger = logging.getLogger("vision-process")
        if hal.HALIsSimulation():
            self.cap = VideoCaptureSim()
        else:
            self.cap = setup_capture(-1)
        self.logger.info(self.cap)
        # Register with Resource so teardown works
        Resource._add_global_resource(self)

    def run(self):
        self.logger.info("Process started")
        with suppress_stdout_stderr():
            while self._run_event.is_set():
                tm = time.time()
                success, image = self.cap.read()
                if success:
                    x, y, w, h, image = self.findTarget(image)
                    self.vision_data_array[:] = [x, y, w, h, tm]
                else:
                    self.vision_data_array[:] = [0.0, 0.0, 0.0, 0.0, tm]

    def findTarget(self, image, drawbox=False):
        # Convert from BGR colourspace to HSV. Makes thresholding easier.
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # #Define the colours to look for (in HSV)
        lower_colour = np.array([40, 200, 20])
        upper_colour = np.array([110, 255, 150])
        # Create a mask that filters out only those colours
        mask = cv2.inRange(hsv_image, lower_colour, upper_colour)
        # Errode and dialate the image to get rid of noise
        kernel = np.ones((4, 4), np.uint8)
        erosion = cv2.erode(mask, kernel, iterations=1)
        dilated = cv2.dilate(erosion, kernel, iterations=1)
        # Get the information for the contours
        _, contours, __ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # sort the contours into a list
        areas = [cv2.contourArea(contour) for contour in contours]
        # and retrieve the largest contour in the list
        cnt = None
        try:
            cnt = contours[np.argmax(areas)]
        except ValueError:
            result_image = image
            return 0.0, 0.0, 0.0, 0.0, result_image


        # get the area of the contour
        area = cv2.contourArea(cnt)
        # get the perimeter of the contour
        perimeter = cv2.arcLength(cnt, True)
        # get a rectangle and then a box around the largest countour
        rect = cv2.minAreaRect(cnt)
        if drawbox:
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            #uncomment this line and remove brackets around box to draw all contours
            #box = [np.int0(cv2.boxPoints(cv2.minAreaRect(contour))) for contour in contours]
            cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
        (xy, wh, rotation_angle) = (rect[0], rect[1], rect[2])
        result_image = image
        # Converting the width and height variables to inbetween -1 and 1
        try:
            (x, y) = xy
            (w, h) = wh
        except ValueError:
            return 0.0, 0.0, 0.0, 0.0, result_image
        if rotation_angle < -45.0 or rotation_angle > 45.0:
            w, h = h, w
        x = ((2 * x) / Vision.video_width) - 1
        y = ((2 * y) / Vision.video_height) - 1
        w = w / Vision.video_width
        h = h / Vision.video_height
        return x, y, w, h, result_image

def setup_capture(device_idx=-1):
    cap = cv2.VideoCapture(device_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, Vision.video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Vision.video_height)
    device_string = str(device_idx)
    if device_idx < 0:
        device_idx = str(0)
    os.system("v4l2-ctl -d /dev/video" + device_idx + " -c exposure_auto=1 -c exposure_auto_priority=0 -c exposure_absolute="
            + str(Vision.video_exposure) + " -c white_balance_temperature_auto=0 -c white_balance_temperature=" + str(Vision.video_white_balance))
    # On the Logitech C920 the following options cannot be set:
    # CAP_PROP_EXPOSURE
    # CAP_PROP_HUE

    # The following can be set:
    # CAP_PROP_BRIGHTNESS
    # CAP_PROP_CONTRAST
    # CAP_PROP_SATURATION
    # CAP_PROP_GAIN
    return cap

class VideoCaptureSim():
    def read(self):
        return False, None

# Allow easy capturing of sample images using same settings as on robot
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Capture sample image.')
    parser.add_argument('--device', help='capture device id', default=-1, type=int)
    parser.add_argument('--video', help='display a live video feed of the capture', action='store_true')
    parser.add_argument('--file', help='capture image to a file', type=str, default=None)
    parser.add_argument('--verbose',
            help='print the values coming back from the findTarget function for a specific file', type=str,
            default=None)
    parser.add_argument('--showfile', help='display a specific file with a bounding box drawn around it',
            type=str, default=None)
    args = parser.parse_args()
    cap = None
    if args.file or args.video:
        cap = setup_capture(args.device)
        print("Brightness: %f" % cap.get(cv2.CAP_PROP_BRIGHTNESS))
        print("Contrast: %f" % cap.get(cv2.CAP_PROP_CONTRAST))
        print("Saturation: %f" % cap.get(cv2.CAP_PROP_SATURATION))
        print("Gain: %f" % cap.get(cv2.CAP_PROP_GAIN))
    if args.file:
        retval, image = cap.read()
        if retval:
            x, y, w, h, image = VisionProcess.findTarget(None, image)
            cv2.imwrite(args.file, image)
    if args.verbose:
        image = cv2.imread(args.verbose, cv2.IMREAD_COLOR)
        x, y, w, h, image = VisionProcess.findTarget(None, image)
        print("x: %f\ny: %f\nwidth: %f\nheight: %f" % (x, y, w, h))
    if args.showfile:
        image = cv2.imread(args.showfile, cv2.IMREAD_COLOR)
        x, y, w, h, image = VisionProcess.findTarget(None, image, drawbox=True)
        cv2.imshow('image',image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    if args.video:
        window = cv2.namedWindow("preview")
        while True:
            retval, image = cap.read()
            img = VisionProcess.findTarget(None, image)
            cv2.imshow("preview", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


# Define a context manager to suppress stdout and stderr.
class suppress_stdout_stderr(object):
    '''
    A context manager for doing a "deep suppression" of stdout and stderr in
    Python, i.e. will suppress all print, even if the print originates in a
    compiled C/Fortran sub-function.
       This will not suppress raised exceptions, since exceptions are printed
    to stderr just before a script exits, and after the context manager has
    exited (at least, I think that is why it lets exceptions through).

    '''
    def __init__(self):
        # Open a pair of null files
        self.null_fds = [os.open(os.devnull, os.O_RDWR) for x in range(2)]
        # Save the actual stdout (1) and stderr (2) file descriptors.
        self.save_fds = (os.dup(1), os.dup(2))

    def __enter__(self):
        # Assign the null pointers to stdout and stderr.
        os.dup2(self.null_fds[0], 1)
        os.dup2(self.null_fds[1], 2)

    def __exit__(self, *_):
        # Re-assign the real stdout/stderr back to (1) and (2)
        os.dup2(self.save_fds[0], 1)
        os.dup2(self.save_fds[1], 2)
        # Close the null files
        os.close(self.null_fds[0])
        os.close(self.null_fds[1])

