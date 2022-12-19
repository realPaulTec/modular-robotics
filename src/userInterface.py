import flet as ft
import trackingV5 as tv5
import cv2

tracker = tv5.systemTrack(0)

def main(page: ft.main):
    cvImage = tracker.mainCycle()[1]
    image = cv2.imencode('.png', cvImage)[1].tostring()
    
    tracker.simpleUI(cvImage)

    page.add(image)    
    page.update()

ft.app(target = main)