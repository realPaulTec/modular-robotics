import kivy
import cv2

from kivy.app import App
from kivy.uix.label import Label
from kivy.clock import Clock
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.graphics.texture import Texture

import trackingV5 as tv5

trackingSystem = tv5.systemTrack(0)

class MyApp(App):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.cvImage = Image()
        print(self.cvImage.texture)

        Clock.schedule_interval(self.callback, 0.05)

    def build(self):
        layout = BoxLayout()
        layout.add_widget(self.cvImage)

        return layout

    def convertCvTexture(self, frame):
        buffer = cv2.flip(frame, 0)
        buffer = buffer.tobytes()

        texture = Texture.create(size = (frame.shape[1], frame.shape[0]), colorfmt = 'bgr')
        texture.blit_buffer(buffer, colorfmt = 'bgr', bufferfmt = 'ubyte')

        print(texture)

        return texture

    def callback(self, *largs):
        components, frame, out = trackingSystem.mainCycle()
        self.cvImage.texture = self.convertCvTexture(frame)

        # trackingSystem.simpleUI(frame)

if __name__ == '__main__':
    MyApp().run()
