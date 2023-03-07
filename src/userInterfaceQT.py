#!../.venv/bin/python3.11

from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *

import trackingV5 as tv5
import threading
import sys
import cv2

cameraIndex = 1

tracker = tv5.systemTrack(1)

class applicationLayout(QWidget):
    def __init__(self):
        super().__init__()

        self.imageLabel = QLabel()

        self.primartLayout = QVBoxLayout()
        self.primartLayout.addWidget(self.imageLabel)

        self.setLayout(self.primartLayout)

    def update(self):
        while True:
            tracker.mainCycle()
            
            interfaceFrame = cv2.cvtColor(tracker.frame, cv2.COLOR_BGR2RGB)
            interfaceFrame = cv2.flip(interfaceFrame, 1)

            self.trackingImage = QImage(interfaceFrame, interfaceFrame.shape[1], interfaceFrame.shape[0], interfaceFrame.strides[0], QImage.Format_RGB888)
            self.imageLabel.setPixmap(QPixmap.fromImage(self.trackingImage))

if __name__ == "__main__":
    application = QApplication([])

    layout = applicationLayout()
    layout.resize(1200, 1200)
    layout.show()

    updateThread = threading.Thread(target = layout.update)
    updateThread.daemon = True

    updateThread.start()

    sys.exit(application.exec())