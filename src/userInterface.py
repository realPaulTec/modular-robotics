import flet as ft
import trackingV5 as tv5
import cv2
import base64
import threading
import multiprocessing


tracker = tv5.systemTrack(0)

class application:
    def __init__(self) -> None:
        self.label = ft.Text(str( 0 ))
        self.cvImage = ft.Image()
        self.counter = 0

        _, self.frame, _ = tracker.mainCycle()

    def build(self, page: ft.main):
        self.page = page

        # Setting the main page up
        self.page.title ='Tracking with TV5'
        self.page.theme_mode = ft.ThemeMode.DARK        
        self.page.padding = 50
        
        # Adding widgets including tracking image
        self.page.add(self.cvImage)
        self.page.add(self.label)
        self.page.update()

        mainThread.start()

    def interfaceUpdate(self):
        global counter, label, page
        
        while True:
            self.counter += 1
            self.label.value = str(self.counter)
            
            # Updating the page to apply changes
            self.page.update()

    def frameUpdate(self):
        # Setting up tracking in thread to avoid lag
        global frame

        while True:
            _, self.frame, _ = tracker.mainCycle()

    def encodingUpdate(self):
        # Converting the frame to b64 for Flet & setting up encoding Thread to avoid lag
        global cvImage, frame

        while True:
            self.cvImage.src_base64 = toBase64(self.frame)

def toBase64(image):
    buffer = cv2.imencode('.png', image)[1]
    buffer = base64.b64encode(buffer).decode('utf-8')
    
    return buffer

mainPage = application()

if __name__ == '__main__':
    # Main thread setup / Starting it later after application build()
    mainThread = threading.Thread(target = mainPage.interfaceUpdate)
    mainThread.daemon = True
    
    # Tracking thread setup
    frameThread = threading.Thread(target = mainPage.frameUpdate)
    frameThread.daemon = True

    frameThread.start()
    
    # Encoding thread setup
    encodingThread = threading.Thread(target = mainPage.encodingUpdate)
    encodingThread.daemon = True

    encodingThread.start()

    # Starting the GUI 
    ft.app(target = mainPage.build, assets_dir = '../assets')
    
# def mainloop():
#     while True:
#         mainPage.interfaceUpdate()