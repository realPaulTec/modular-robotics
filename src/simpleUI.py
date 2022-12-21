import trackingV5 as tv5
import cv2

tracker = tv5.systemTrack(0)

# myImage = cv2.imread('../assets/images/frame_screenshot_19.12.2022.png')

while True:
    tracker.mainCycle()
    tracker.simpleUI(tracker.frame)