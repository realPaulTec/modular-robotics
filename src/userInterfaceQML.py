#!../.venv/bin/python3.11

from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtQml import *
from PySide6.QtQuick import *
import trackingV5 as tv5
import advancedLiDAR as avL
import numpy as np
import threading
import subprocess
import time
import sys
import cv2

# sudo apt-get install v4l-utils

np.set_printoptions(threshold=sys.maxsize)

RECONNECTS = 2
LiDAR_ACTIVE = False

try:
    # Get system cameras and show them to the user.
    result = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True)
    print("Available Video Devices:\n")

    for camera in result.stdout.split("\n"):
        print(camera.strip().split("USB Camera (")[-1].strip(")"))

    # Get the users desired video device
    cameraIndex = input("Video Device: ")

    # Exit if the user input an incorrect camera index!
    try:
        tracker = tv5.systemTrack(cameraIndex)

    except IOError:
        print(f"ERROR: Cannot open Video Device at: {cameraIndex}")

        sys.exit()

    for i in range(RECONNECTS - 1):
        try:
            currentLiDAR = avL.LiDAR()
            LiDAR_ACTIVE = True

        except Exception:
            print(f'ERROR: attempting to reconnect | {i} / {RECONNECTS}')
            time.sleep(2)

            if i == (RECONNECTS - 1):
                break

            else: continue

        else:
            break

except KeyboardInterrupt:
    print('\n\nExiting on KeyboardInterrupt!')

    sys.exit()


class Backend:
    def __init__(self, src, imageProvider) -> None:
        super().__init__()
        self.engine = QQmlApplicationEngine()
        
        # Setting up signal handeler
        self.signalHandeler = SignalHandeler()
        self.engine.rootContext().setContextProperty('signalHandeler', self.signalHandeler)
        
        # Setting up image provider for video feed
        self.imageProvider = imageProvider
        self.engine.addImageProvider('imageprovider', self.imageProvider)

        # Cycling once for image update
        self.interfaceCycle(kill = True)
        
        # Loading user interface from QML file
        self.engine.load(src)

        # Setting up window context
        self.windowContext = self.engine.rootObjects()[0]

        # Setting up timer for image updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.interfaceCycle)
        self.timer.start(0)

    def interfaceCycle(self, kill = False):
        tracker.mainCycle()

        if LiDAR_ACTIVE == True and self.signalHandeler.dropdownSelection == "LiDAR":
            # Getting LiDAR image!
            lidarData = currentLiDAR.graphingMatrix
            
            # Fixing the image for QML integration
            lidarData = np.array(lidarData, dtype=np.float32)
            lidarData = np.interp(lidarData, (0, 8), (0, 255)).astype(np.float32)
            lidarData = np.rint(lidarData).astype(np.uint8)

            lidarData = cv2.resize(lidarData, (720, 720), interpolation=cv2.INTER_LINEAR)
            lidarData = cv2.merge([lidarData, lidarData, lidarData])

            interfaceFrame = lidarData

        else:
            currentFrame = np.array(tracker.frameHistory.get(self.signalHandeler.dropdownSelection, tracker.frameHistory["final"]))

            # Fixing the image for QML integration
            interfaceFrame = cv2.cvtColor(currentFrame, cv2.COLOR_BGR2RGB)
            interfaceFrame = cv2.flip(interfaceFrame, 1)


        trackingImage = QImage(interfaceFrame, interfaceFrame.shape[1], interfaceFrame.shape[0], interfaceFrame.strides[0], QImage.Format_RGB888)
        self.imageProvider.updateImage(trackingImage, 'trackingImage')

        self.set_tracker_values()
        
        # Killing the cycle on the first round
        if kill != True:
            textAmountComponents = self.windowContext.findChild(QObject, "textAmountComponents")
            
            if textAmountComponents is not None:
                textAmountComponents.setProperty("text", str(len(tracker.trackComponents)))
            
            frequencyComponents = self.windowContext.findChild(QObject, "frequencyComponents")
            
            if frequencyComponents is not None:
                frequencyComponents.setProperty("text", f"{tracker.closestFrequency}Hz")

    def set_tracker_values(self):
        # Set all tracking system values to the ones from the Signal handler
        tracker.useMask = self.signalHandeler.maskSelection
        tracker.erosionSteps = self.signalHandeler.erosionSteps
        tracker.targetChannel = int(self.signalHandeler.targetChannel)

        tracker.dilationSteps[0] = self.signalHandeler.dilationStepsBlue
        tracker.dilationSteps[1] = self.signalHandeler.dilationStepsGreen
        tracker.dilationSteps[2] = self.signalHandeler.dilationStepsRed
        tracker.dilationSteps[3] = self.signalHandeler.dilationStepsBright

        # Providing tracking threshold data
        tracker.thresholdRedMax = self.signalHandeler.thresholdRedMax
        tracker.thresholdRedMin = self.signalHandeler.thresholdRedMin

        tracker.thresholdGreenMax = self.signalHandeler.thresholdGreenMax
        tracker.thresholdGreenMin = self.signalHandeler.thresholdGreenMin

        tracker.thresholdBlueMax = self.signalHandeler.thresholdBlueMax
        tracker.thresholdBlueMin = self.signalHandeler.thresholdBlueMin

        tracker.thresholdBrightMax = self.signalHandeler.thresholdBrightMax
        tracker.thresholdBrightMin = self.signalHandeler.thresholdBrightMin

        tracker.MAX_MATRIX_DIFFERENCE = self.signalHandeler.maxDifference
        tracker.MAX_LIFETIME_SECONDS = self.signalHandeler.lifetime

        tracker.wCoordinates = self.signalHandeler.wCoordinates
        tracker.wBounds = self.signalHandeler.wBounds
        tracker.wArea = self.signalHandeler.wArea

        tracker.frequency = self.signalHandeler.frequency
        tracker.frequencyBuffer = self.signalHandeler.frequencyBuffer        


class SignalHandeler(QObject):
    def __init__(self):
        super().__init__()
        self.targetChannel = 0
        
        self.thresholdRedMax = 255
        self.thresholdRedMin = 230
        self.dilationStepsRed = 6

        self.thresholdGreenMax = 255
        self.thresholdGreenMin = 230
        self.dilationStepsGreen = 6

        self.thresholdBlueMax = 255
        self.thresholdBlueMin = 230
        self.dilationStepsBlue = 6

        self.thresholdBrightMax = 255
        self.thresholdBrightMin = 200
        self.dilationStepsBright = 6

        self.dropdownSelection = "final"

        self.LiDARselection = False

        self.frameHistory = [
            "LiDAR",
            "camera feed",
            "color corrected",
            "masked frame",
            "blue image",
            "green image",
            "red image",
            "white image",
            "thresholding blue",
            "thresholding green",
            "thresholding red",
            "thresholding white",
            "combination",
            "tracking image",
            "final"
        ]

        self.maskSelection = False
        self.erosionSteps = 4
        self.lifetime = 1.0

        self.maxDifference = 140

        self.wCoordinates = 0.6
        self.wBounds = 1.0
        self.wArea = 0.8

        self.frequency = 3
        self.frequencyBuffer = 2

    @Slot(str)
    def radioSignal(self, incoming):
        self.targetChannel = incoming

    @Slot(str)
    def dropDown(self, incoming):
        self.dropdownSelection = self.frameHistory[int(incoming)]

    @Slot(str)
    def buttonMaskSelection(self, incoming):
        self.maskSelection = (int(incoming) == 1)

    @Slot(str)
    def selectLiDAR(self, incoming):
        self.LiDARselection = (int(incoming) == 1)

        print(self.LiDARselection)

    @Slot(str)
    def erosionSelection(self, incoming):
        self.erosionSteps = int(incoming)

    @Slot(str)
    def redSliderMaxSignal(self, incoming):
        self.thresholdRedMax = round(float(incoming))

    @Slot(str)
    def redSliderMinSignal(self, incoming):
        self.thresholdRedMin = round(float(incoming))

    @Slot(str)
    def redSetDilation(self, incoming):
        self.dilationStepsRed = int(incoming)

    @Slot(str)
    def greenSliderMaxSignal(self, incoming):
        self.thresholdGreenMax = round(float(incoming))

    @Slot(str)
    def greenSliderMinSignal(self, incoming):
        self.thresholdGreenMin = round(float(incoming))

    @Slot(str)
    def greenSetDilation(self, incoming):
        self.dilationStepsGreen = int(incoming)

    @Slot(str)
    def blueSliderMaxSignal(self, incoming):
        self.thresholdBlueMax = round(float(incoming))

    @Slot(str)
    def blueSliderMinSignal(self, incoming):
        self.thresholdBlueMin = round(float(incoming))

    @Slot(str)
    def blueSetDilation(self, incoming):
        self.dilationStepsBlue = int(incoming)

    @Slot(str)
    def brightSliderMaxSignal(self, incoming):
        self.thresholdBrightMax = round(float(incoming))

    @Slot(str)
    def brightSliderMinSignal(self, incoming):
        self.thresholdBrightMin = round(float(incoming))

    @Slot(str)
    def brightSetDilation(self, incoming):
        self.dilationStepsBright = int(incoming)

    @Slot(str)
    def setMaxDifference(self, incoming):
        self.maxDifference = int(incoming)

    @Slot(str)
    def setLifetime(self, incoming):
        self.lifetime = int(incoming) / 100

    @Slot(str)
    def setWeightCoordinates(self, incoming):
        self.wCoordinates = float(incoming)

    @Slot(str)
    def setWeightBounds(self, incoming):
        self.wBounds = float(incoming)

    @Slot(str)
    def setWeightArea(self, incoming):
        self.wArea = float(incoming)

    @Slot(str)
    def setFrequency(self, incoming):
        self.frequency = int(incoming)

    @Slot(str)
    def setFrequencyBuffer(self, incoming):
        self.frequencyBuffer = int(incoming)


class ImageProvider(QQuickImageProvider):
    def __init__(self, type):
        super().__init__(type, flags = QQmlImageProviderBase.Flags())
        self.trackingImage = QImage()
    
    def updateImage(self, frame, id):
        if id == 'trackingImage':
            self.trackingImage = frame

    def requestImage(self, id, size, requestedSize):
        if id == 'trackingImage.png' or id == 'secondaryTrackingImage.png':
            return self.trackingImage


if __name__ == "__main__":    
    # Application setup
    application = QApplication([])
    applicationBackend = Backend('src/layout.qml', ImageProvider(QQmlImageProviderBase.ImageType.Image))

    # Setting up and starting the scan thread for the LiDAR
    if LiDAR_ACTIVE == True:
        scanThread = threading.Thread(target = currentLiDAR.scan_thread)
        scanThread.daemon = True
        scanThread.start()

    # Exiting on window close
    sys.exit(application.exec())
