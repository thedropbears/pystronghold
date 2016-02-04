from components.vision import Vision

import csv
import cv2
import multiprocessing

def test_sample_images():
    variables = ['x', 'y', 'w', 'h']
    vision = Vision()
    with open('sample_img/tests.csv', 'r') as csvfile:
        # filename, x, y, w, h
        testreader = csv.reader(csvfile, delimiter=',')
        for sample in testreader:
            image = cv2.imread('sample_img/' + sample[0])
            # Rescale if necessary
            scaled = cv2.resize(image, (Vision.video_width, Vision.video_height))
            results = vision._vision_process.findTarget(scaled)
            yield find_target, sample[0], results[:-1], sample[1:], [0.05, 0.05]  # Don't send the return image
    vision.free()

def find_target(filename, result, desired, deltas):
    message = filename + " - %s\nExpected: %s +/- %s\nReceived: %s"
    assert abs(result[0] - float(desired[0])) < deltas[0], message % ("x position", str(desired[0]), str(deltas[0]), str(result[0]))
    assert abs(result[1] - float(desired[1])) < deltas[0], message % ("y position", str(desired[1]), str(deltas[0]), str(result[1]))
    assert abs(result[2] - float(desired[2])) < deltas[1] * float(desired[2]), message % ("width", str(desired[2]), str(deltas[1] * float(desired[2])), str(result[2]))
    assert abs(result[3] - float(desired[3])) < deltas[1] * float(desired[3]), message % ("height", str(desired[3]), str(deltas[1] * float(desired[3])), str(result[3]))
