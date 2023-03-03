#!../.venv/bin/python3.11

import flet as ft
import trackingV5 as tv5
import cv2
import base64
import threading

tracker = tv5.systemTrack(-1)


class application:
    def __init__(self) -> None:
        _, self.frame, _ = tracker.mainCycle()
        self.cvImage = ft.Image()

    def controls(self):
        self.label = ft.Text(font_family='URWgothic', size=16)

        RBS = ft.ButtonStyle(color=ft.colors.RED_ACCENT_700,
                             surface_tint_color=ft.colors.RED)
        GBS = ft.ButtonStyle(color=ft.colors.GREEN_ACCENT_700,
                             surface_tint_color=ft.colors.GREEN)
        BBS = ft.ButtonStyle(color=ft.colors.BLUE_ACCENT_700,
                             surface_tint_color=ft.colors.BLUE)

        self.redButton = ft.ElevatedButton(
            text='R', on_click=self.changeChannelRed, style=RBS)
        self.greenButton = ft.ElevatedButton(
            text='G', on_click=self.changeChannelGreen, style=GBS)
        self.blueButton = ft.ElevatedButton(
            text='B', on_click=self.changeChannelBlue, style=BBS)

        self.divider = ft.VerticalDivider(
            width=9, thickness=3, color=ft.colors.WHITE)
        self.divider_2 = ft.VerticalDivider(
            width=100, thickness=3, color=ft.colors.WHITE)

        self.labelSaturation = ft.Text(
            value='Saturation', font_family='URWgothic', size=16)
        self.labelContrast = ft.Text(
            value='Contrast', font_family='URWgothic', size=16)
        self.labelBrightness = ft.Text(
            value='Brightness', font_family='URWgothic', size=16)

    def changeChannelBlue(self, *args):
        tracker.targetChannel = 0

        self.redButton.disabled = False
        self.greenButton.disabled = False
        self.blueButton.disabled = True

    def changeChannelGreen(self, *args):
        tracker.targetChannel = 1

        self.redButton.disabled = False
        self.greenButton.disabled = True
        self.blueButton.disabled = False

    def changeChannelRed(self, *args):
        tracker.targetChannel = 2

        self.redButton.disabled = True
        self.greenButton.disabled = False
        self.blueButton.disabled = False

    def build(self, page: ft.main):
        self.controls()

        self.page = page

        self.saturationSlider = ft.Slider(
            min=0, max=2, value=1.2)
        self.contrastSlider = ft.Slider(
            min=0, max=127, value=127)
        self.brightnessSlider = ft.Slider(
            min=0, max=255, value=255)

        self.displayFrameDropdown = ft.Dropdown(
            width=250,
            options= [
                ft.dropdown.Option("camera feed"),
                ft.dropdown.Option("color corrected"),
                ft.dropdown.Option("masked frame"),
                ft.dropdown.Option("blue image"),
                ft.dropdown.Option("green image"),
                ft.dropdown.Option("red image"),
                ft.dropdown.Option("white image"),
                ft.dropdown.Option("threshholding blue"),
                ft.dropdown.Option("threshholding green"),
                ft.dropdown.Option("threshholding red"),
                ft.dropdown.Option("threshholding white"),
                ft.dropdown.Option("combination"),
                ft.dropdown.Option("tracking image"),
                ft.dropdown.Option("final")
            ]
        )

        self.displayFrameDropdown.value = "final"

        # Setting the main page up
        self.page.title = 'Tracking with TV5'
        self.page.theme_mode = ft.ThemeMode.DARK
        self.page.padding = 50
        self.page.fonts = {
            'URWgothic': f'./fonts/URWGothic-Book.otf'
        }

        self.changeChannelBlue()

        col_Sliders = ft.Column(
            controls=[self.saturationSlider, self.contrastSlider, self.brightnessSlider])

        col_labels = ft.Column(controls=[self.labelSaturation, self.labelContrast, self.labelBrightness],
                               spacing=35, alignment=ft.MainAxisAlignment.CENTER, horizontal_alignment=ft.CrossAxisAlignment.CENTER, width=180)

        row_RGB = ft.Row(
            controls=[self.redButton, self.greenButton, self.blueButton])

        col_RGB = ft.Column(controls=[self.label, row_RGB], alignment=ft.MainAxisAlignment.CENTER,
                            horizontal_alignment=ft.CrossAxisAlignment.CENTER, spacing=20)

        row_0 = ft.Row(controls=[col_Sliders, col_labels, col_RGB])
        row_1 = ft.Row(controls=[self.displayFrameDropdown])
        col_Main = ft.Column(controls=[self.cvImage, row_0, row_1], alignment=ft.MainAxisAlignment.CENTER)

        self.mainContainer = ft.Container(
            content=col_Main, margin=20, padding=20, bgcolor=ft.colors.BLACK12, border_radius=20, alignment=ft.alignment.center)

        mainRow = ft.Row(controls=[self.mainContainer], alignment=ft.MainAxisAlignment.CENTER, vertical_alignment=ft.CrossAxisAlignment.CENTER)

        # Adding widgets including the video feed
        self.page.add(mainRow)
        self.page.update()

        mainThread.start()
        frameThread.start()
        encodingThread.start()

    def interfaceUpdate(self):
        global counter, label, page, tracker

        while True:
            if tracker.targetChannel == 0:
                colorChannel = 'Blue'
            elif tracker.targetChannel == 1:
                colorChannel = 'Green'
            elif tracker.targetChannel == 2:
                colorChannel = 'Red'

            self.label.value = 'Targeting %s channel' % colorChannel

            # Updating the page to apply changes
            self.page.update()

    def frameUpdate(self):
        # Setting up tracking in thread to avoid lag
        global tracker, frame, saturationSlider, contrastSlider, brightnessSlider

        while True:
            tracker.contrastAdjustment = self.contrastSlider.value
            tracker.saturationAdjustment = self.saturationSlider.value
            tracker.brightnessAdjustment = self.brightnessSlider.value

            _, self.frame, self.frameHistory = tracker.mainCycle()

            # Select which frame to show
            self.frame = self.frameHistory[str(self.displayFrameDropdown.value)]

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
    mainThread = threading.Thread(target=mainPage.interfaceUpdate)
    mainThread.daemon = True

    # Tracking thread setup
    frameThread = threading.Thread(target=mainPage.frameUpdate)
    frameThread.daemon = True

    # Encoding thread setup
    encodingThread = threading.Thread(target=mainPage.encodingUpdate)
    encodingThread.daemon = True

    # Starting the GUI
    ft.app(target=mainPage.build, assets_dir='../assets')
