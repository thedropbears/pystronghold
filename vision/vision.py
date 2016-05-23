import cv2
import numpy as np
import argparse
import os
import re
import time
import logging
from networktables import NetworkTable

def findTarget(image):
    height = image.shape[0]
    width = image.shape[1]
    # Convert from BGR colourspace to HSV. Makes thresholding easier.
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Define the colours to look for (in HSV)
    # Use values straight from GIMP
    lower_colour = np.array([80 * 0.5, 70 * 255 / 100, 8 * 255 / 100])
    upper_colour = np.array([220 * 0.5, 100 * 255 / 100, 63 * 255 / 100])
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
        return 0.0, 0.0, 0.0, 0.0, image

    # Draw the contours
    cv2.drawContours(image, contours, 0, (255, 0, 0), 1)

    # get the area of the contour
    area = cv2.contourArea(cnt)
    if area / width / height > 0.05:
        return 0.0, 0.0, 0.0, 0.0, image
    # get a rectangle and then a box around the largest countour
    rect = cv2.minAreaRect(cnt)

    # Draw the box
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
    (xy, wh, rotation_angle) = (rect[0], rect[1], rect[2])
    # Converting the width and height variables to inbetween -1 and 1
    try:
        (x, y) = xy
        (w, h) = wh
    except ValueError:
        return 0.0, 0.0, 0.0, 0.0, image
    if rotation_angle < -45.0 or rotation_angle > 45.0:
        w, h = h, w
    # Draw the centre point
    cv2.circle(image, (int(x), int(y)), 2, (0, 0, 255), 2)
    x = ((2 * x) / width) - 1
    y = ((2 * y) / height) - 1
    w = w / width
    h = h / height
    # Send the results to NetworkTables
    # TODO!!

    return x, y, w, h, image

class NTWrapper:  # pragma: no cover
    def __init__(self):
        NetworkTable.setIPAddress('127.0.0.1')
        NetworkTable.setClientMode()
        NetworkTable.initialize()
        self.nt = NetworkTable.getTable("vision")

    def findTargetNetworkTables(self, image):
        x, y, w, h, img = findTarget(image)
        # TODO - send to network tables
        self.nt.putDouble('x', x)
        self.nt.putDouble('y', y)
        self.nt.putDouble('w', w)
        self.nt.putDouble('h', h)
        self.nt.putDouble('time', time.time())
        return img


def init_filter():  # pragma: no cover
    ntw = NTWrapper()
    return ntw.findTargetNetworkTables


def setCaptureParameters(device,
                         mjpg_config_file='mjpg-streamer'):  # pragma: no cover
    if not os.path.exists(device):
        raise Exception("No such video device: %s" % device)

    logger = logging.getLogger("vision")

    root_device = os.path.realpath(device)

    # Try to find the settings from the mjpg-streamer config file
    v4l2_str = "v4l2-ctl -d %s" % device
    try:
        with open(mjpg_config_file) as f:
            for line in f:
                # Follow symlinks to see if they point at the same device
                if "--device" in line:
                    p = re.compile('--device ([^ \t\n\r\f\v"\']+)')
                    if os.path.realpath(p.search(line).group(1)) == root_device:
                        # A bunch of regexes to find the parameters
                        p = re.compile('-ex ([0-9]+)')
                        if p.search(line):
                            v4l2_str += (" -c exposure_auto=1 -c exposure_absolute=%s"
                                         % p.search(line).group(1))
                            logger.info("Found exposure setting: %s"
                                        % p.search(line).group(1))
                        p = re.compile('-br ([0-9]+)')
                        if p.search(line):
                            v4l2_str += " -c brightness=%s" % p.search(line).group(1)
                            logger.info("Found brightness setting: %s"
                                        % p.search(line).group(1))
                        p = re.compile('-co ([0-9]+)')
                        if p.search(line):
                            v4l2_str += " -c contrast=%s" % p.search(line).group(1)
                            logger.info("Found contrast setting: %s"
                                        % p.search(line).group(1))
                        p = re.compile('-sa ([0-9]+)')
                        if p.search(line):
                            v4l2_str += " -c saturation=%s" % p.search(line).group(1)
                            logger.info("Found saturation setting: %s"
                                        % p.search(line).group(1))
        os.system(v4l2_str)
    except:
        raise  # pass  # File not found

# Allow easy capturing of sample images using same settings as on robot
if __name__ == "__main__":
    logger = logging.getLogger("vision")
    parser = argparse.ArgumentParser(description='Capture sample image.')
    parser.add_argument('--device', help='capture device', default='/dev/video0')
    parser.add_argument('--video', help='display a live video feed of the capture', action='store_true')
    parser.add_argument('--file', help='capture image to a file', type=str, default=None)
    parser.add_argument('--verbose',
            help='print the values coming back from the findTarget function for a specific file', type=str,
            default=None)
    parser.add_argument('--showfile', help='display a specific file with a bounding box drawn around it',
            type=str, default=None)
    args = parser.parse_args()

    logging.basicConfig(level=20)  # Show info messages

    cap = None
    if args.file or args.video:
        setCaptureParameters(args.device)
        cap = cv2.VideoCapture(args.device)
        logger.info("Brightness: %f" % cap.get(cv2.CAP_PROP_BRIGHTNESS))
        logger.info("Contrast: %f" % cap.get(cv2.CAP_PROP_CONTRAST))
        logger.info("Saturation: %f" % cap.get(cv2.CAP_PROP_SATURATION))
        logger.info("Exposure: %f" % cap.get(cv2.CAP_PROP_EXPOSURE))
    if args.file:
        retval, image = cap.read()
        if retval:
            x, y, w, h, image = findTarget(image)
            cv2.imwrite(args.file, image)
    if args.verbose:
        image = cv2.imread(args.verbose, cv2.IMREAD_COLOR)
        x, y, w, h, image = findTarget(image)
        # print("x: %f\ny: %f\nwidth: %f\nheight: %f" % (x, y, w, h))
    if args.showfile:
        image = cv2.imread(args.showfile, cv2.IMREAD_COLOR)
        x, y, w, h, image = findTarget(image)
        cv2.imshow('image', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    if args.video:
        window = cv2.namedWindow("preview")
        while True:
            retval, image = cap.read()
            x, y, w, h, image = findTarget(image)
            cv2.imshow("preview", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
