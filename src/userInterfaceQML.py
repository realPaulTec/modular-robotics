#!../.venv/bin/python3.11

from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtQml import *

from PySide6.QtQuick import *

import trackingV5 as tv5
import threading
import sys
import cv2


cameraIndex = 1
tracker = tv5.systemTrack(-1)

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

    def interfaceCycle(self, kill = False):
        while True:
            tracker.mainCycle()

            currentFrame = tracker.frameHistory[self.signalHandeler.dropdownSelection]

            # Fixing the image for QML integration
            interfaceFrame = cv2.cvtColor(currentFrame, cv2.COLOR_BGR2RGB)
            interfaceFrame = cv2.flip(interfaceFrame, 1)

            trackingImage = QImage(interfaceFrame, interfaceFrame.shape[1], interfaceFrame.shape[0], interfaceFrame.strides[0], QImage.Format_RGB888)
            self.imageProvider.updateImage(trackingImage, 'trackingImage')

            tracker.useMask = self.signalHandeler.maskSelection
            tracker.erosionSteps = self.signalHandeler.erosionSteps
            tracker.targetChannel = int(self.signalHandeler.targetChannel)

            tracker.dialationSteps[0] = self.signalHandeler.dialationStepsBlue
            tracker.dialationSteps[1] = self.signalHandeler.dialationStepsGreen
            tracker.dialationSteps[2] = self.signalHandeler.dialationStepsRed
            tracker.dialationSteps[3] = self.signalHandeler.dialationStepsBright

            # Providing tracking threshhold data
            tracker.threshholdRedMax = self.signalHandeler.threshholdRedMax
            tracker.threshholdRedMin = self.signalHandeler.threshholdRedMin

            tracker.threshholdGreenMax = self.signalHandeler.threshholdGreenMax
            tracker.threshholdGreenMin = self.signalHandeler.threshholdGreenMin

            tracker.threshholdBlueMax = self.signalHandeler.threshholdBlueMax
            tracker.threshholdBlueMin = self.signalHandeler.threshholdBlueMin

            tracker.threshholdBrightMax = self.signalHandeler.threshholdBrightMax
            tracker.threshholdBrightMin = self.signalHandeler.threshholdBrightMin
            
            # Killing the cycle on the first round
            if kill == True:
                break

class SignalHandeler(QObject):
    radioButtonSignal = Signal(int)

    redSliderMaxSignal = Signal(int)
    redSliderMinSignal = Signal(int)

    greenSliderMaxSignal = Signal(int)
    greenSliderMinSignal = Signal(int)

    blueSliderMaxSignal = Signal(int)
    blueSliderMinSignal = Signal(int)

    brightSliderMaxSignal = Signal(int)
    brightSliderMinSignal = Signal(int)

    def __init__(self):
        super().__init__()
        self.targetChannel = 0
        
        self.threshholdRedMax = 255
        self.threshholdRedMin = 230
        self.dialationStepsRed = 6

        self.threshholdGreenMax = 255
        self.threshholdGreenMin = 230
        self.dialationStepsGreen = 6

        self.threshholdBlueMax = 255
        self.threshholdBlueMin = 230
        self.dialationStepsBlue = 6

        self.threshholdBrightMax = 255
        self.threshholdBrightMin = 200
        self.dialationStepsBright = 6

        self.dropdownSelection = "final"

        self.frameHistory = [
            "camera feed",
            "color corrected",
            "masked frame",
            "blue image",
            "green image",
            "red image",
            "white image",
            "threshholding blue",
            "threshholding green",
            "threshholding red",
            "threshholding white",
            "combination",
            "tracking image",
            "final"
        ]

        self.maskSelection = True
        self.erosionSteps = 4

    @Slot(str)
    def radioSignal(self, incoming):
        self.targetChannel = incoming

    @Slot(str)
    def dropDown(self, incoming):
        self.dropdownSelection = self.frameHistory[int(incoming)]
        print(self.dropdownSelection)

    @Slot(str)
    def buttonMaskSelection(self, incoming):
        self.maskSelection = (int(incoming) == 1)

    @Slot(str)
    def erosionSelection(self, incoming):
        self.erosionSteps = int(incoming)

    @Slot(str)
    def redSliderMaxSignal(self, incoming):
        self.threshholdRedMax = round(float(incoming))

    @Slot(str)
    def redSliderMinSignal(self, incoming):
        self.threshholdRedMin = round(float(incoming))

    @Slot(str)
    def redSetDialation(self, incoming):
        self.dialationStepsRed = int(incoming)

    @Slot(str)
    def greenSliderMaxSignal(self, incoming):
        self.threshholdGreenMax = round(float(incoming))

    @Slot(str)
    def greenSliderMinSignal(self, incoming):
        self.threshholdGreenMin = round(float(incoming))

    @Slot(str)
    def greenSetDialation(self, incoming):
        self.dialationStepsGreen = int(incoming)

    @Slot(str)
    def blueSliderMaxSignal(self, incoming):
        self.threshholdBlueMax = round(float(incoming))

    @Slot(str)
    def blueSliderMinSignal(self, incoming):
        self.threshholdBlueMin = round(float(incoming))

    @Slot(str)
    def blueSetDialation(self, incoming):
        self.dialationStepsBlue = int(incoming)

    @Slot(str)
    def brightSliderMaxSignal(self, incoming):
        self.threshholdBrightMax = round(float(incoming))

    @Slot(str)
    def brightSliderMinSignal(self, incoming):
        self.threshholdBrightMin = round(float(incoming))

    @Slot(str)
    def brightSetDialation(self, incoming):
        self.dialationStepsBright = int(incoming)


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

    # Creating and starting the update thread
    updateThread = threading.Thread(target = applicationBackend.interfaceCycle)
    updateThread.daemon = True

    updateThread.start()

    # Exiting on window close
    sys.exit(application.exec())
