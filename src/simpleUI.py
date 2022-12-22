import trackingV5 as tv5
import cv2

tracker = tv5.systemTrack(0)

while True:
    tracker.mainCycle()
    tracker.simpleUI(tracker.frame)