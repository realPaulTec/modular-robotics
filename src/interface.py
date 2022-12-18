import kivy
import cv2

from kivy.app import App
from kivy.clock import Clock
from kivy.uix.image import Image
from kivy.uix.gridlayout import GridLayout
from kivy.uix.widget import Widget
from kivy.lang.builder import Builder
from kivy.graphics.texture import Texture
from kivy.properties import ObjectProperty

import trackingV5 as tv5

trackingSystem = tv5.systemTrack(0)

class MainGridLayout(Widget):
    pass

class UserInterface(App):
    cvImageTexture = ObjectProperty(None)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_interval(self.callback, 0.05)

    def build(self):
        # Building the MGL
        return MainGridLayout()

    def convertCvTexture(self, frame):
        buffer = cv2.flip(frame, 0)
        buffer = buffer.tobytes()

        texture = Texture.create(size = (frame.shape[1], frame.shape[0]), colorfmt = 'bgr')
        texture.blit_buffer(buffer, colorfmt = 'bgr', bufferfmt = 'ubyte')

        return texture

    def callback(self, *largs):
        # Reading the opencv frame
        components, frame, out = trackingSystem.mainCycle()

        # Converting opencv frame to Kivy texture
        self.cvImageTexture = self.convertCvTexture(frame)

if __name__ == '__main__':
    UserInterface().run()
