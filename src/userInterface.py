import flet as ft
import trackingV5 as tv5
import cv2
import base64
import threading
import multiprocessing


tracker = tv5.systemTrack(0)

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

        # Setting the main page up
        self.page.title ='Tracking with TV5'
        self.page.theme_mode = ft.ThemeMode.DARK        
        self.page.padding = 50
        
        # Adding widgets including tracking image
        self.page.add(self.cvImage)
        self.page.add(self.label)
        self.page.update()
        
        mainloop()

    def update(self):
        # Converting the frame to b64 for Flet
        self.cvImage.src_base64 = toBase64(self.frame)
        
        self.counter += 1
        self.label.value = str(self.counter)
        
        # Updating the page to apply changes
        self.page.update()

    def frameUpdate(self):
        # Setting up tracking in thread to avoid encoding lag
        global frame

        while True:
            _, self.frame, _ = tracker.mainCycle()

def mainloop():
    while True:
        mainPage.update()

mainPage = application()

if __name__ == '__main__':
    # Setting up the thread for tracking
    mainThread = threading.Thread(target = mainPage.frameUpdate)
    mainThread.daemon = True

    mainThread.start()

    # Starting the GUI 
    ft.app(target = mainPage.build, assets_dir = '../assets')
    
