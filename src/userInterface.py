import flet as ft
import trackingV5 as tv5
import numpy as np
import cv2
import base64
import io
import multiprocessing
import time


tracker = tv5.systemTrack(0)
myImage = cv2.imread('../assets/images/frame_screenshot_19.12.2022.png')

def toBase64(image):
    buffer = cv2.imencode('.png', image)[1]
    buffer = base64.b64encode(buffer).decode('utf-8')
    
    return buffer

def save(image):
    buffer = cv2.imencode('.png', image)[1].tostring()
    cv2.imwrite('../assets/images/cframe.png', buffer)

class application:
    def __init__(self) -> None:
        self.label = ft.Text(str( 0 ))
        self.cvImage = ft.Image()
        self.counter = 0

        _, self.frame, _ = tracker.mainCycle()

    def build(self, page: ft.main):
        self.page = page

        # Setup
        self.page.title ='Tracking with TV5'
        self.page.theme_mode = ft.ThemeMode.DARK        
        self.page.padding = 50
        
        self.page.add(self.cvImage)
        self.page.add(self.label)
        self.page.update()
        
        mainloop()

    def update(self,):
        _, self.frame, _ = tracker.mainCycle()

        self.cvImage.src_base64 = toBase64(self.frame)
        
        self.counter += 1
        self.label.value = str(self.counter)
        
        self.page.update()

mainPage = application()

def mainloop():
    while True:
        mainPage.update()

ft.app(target = mainPage.build, assets_dir = '../assets')

trackingProcess.terminate()