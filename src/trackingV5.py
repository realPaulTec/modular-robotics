#!../.venv/bin/python3.11

import numpy as np
import time
import cv2

# Roadmap:
# Proper GUI


class systemTrack:
    def __init__(self, vcd, targetChannel=0, aqqExtents=(100, 100), saturationAdjustment=1.2, contrastAdjustment=1.02, brightnessAdjustment=0) -> None:
        # Setting up video capture device (camera)
        self.video = cv2.VideoCapture(vcd)
        self.videoDimensions = (self.video.get(cv2.CAP_PROP_FRAME_WIDTH), self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.acquisitionExtents = aqqExtents
        self.targetChannel = targetChannel
        self.saturationAdjustment = saturationAdjustment
        self.contrastAdjustment = contrastAdjustment
        self.brightnessAdjustment = brightnessAdjustment

        self.frame = None
        self.refined_components = []

        self.threshholdRedMax = 255
        self.threshholdRedMin = 230

        self.threshholdGreenMax = 255
        self.threshholdGreenMin = 230

        self.threshholdBlueMax = 255
        self.threshholdBlueMin = 230

        self.threshholdBrightMax = 255
        self.threshholdBrightMin = 200

        self.dialationSteps = [6, 6, 6, 6]

        self.erosionSteps = 4

        self.useMask = True
        self.maskBuffer = 40

        self.frameHistory = {
            "camera feed" : None,
            "color corrected" : None,
            "masked frame" : None,
            "blue image" : None,
            "green image" : None,
            "red image" : None,
            "white image" : None,
            "threshholding blue" : None,
            "threshholding green" : None,
            "threshholding red" : None,
            "threshholding white" : None,
            "combination" : None,
            "tracking image" : None,
            "final" : None
        }

    def mainCycle(self):
        # Reading the frame displated by the system camera
        stream = self.video.read()
        frame = stream[1]

        self.frameHistory["camera feed"] = frame

        if len(self.refined_components) > 0:
            # Using first track coordinates as mask coordinates
            (x, y) = self.refined_components[0][1]
            coordinates = (int(x), int(y))

            # Using first track dimensions as mask radius
            (width, height) = self.refined_components[0][2]
            extents = (int(width), int(height))
        else:
            # Using video center as mask coordinates
            coordinates = (int((self.videoDimensions[0] / 2) - (self.acquisitionExtents[0] / 2)), int((self.videoDimensions[1] / 2) - (self.acquisitionExtents[1] / 2)))

            # Using standard acquisition radius as mask radius
            extents = self.acquisitionExtents

        self.refined_components, self.frame = self.trackCycle(frame, extents, coordinates)

        return self.frame, self.frameHistory, self.refined_components 

    def trackCycle(self, frame, mExtents, mCoordinates):
        # Stage one color correction
        frame = self.adjustSaturation(frame, self.saturationAdjustment)

        # CAUSING PROBLEMS:
        # frame = self.adjustContrast(frame, self.contrastAdjustment, self.brightnessAdjustment)
        
        self.frameHistory["color corrected"] = frame

        # Applying mask with my own function
        if self.useMask == True:
            maskedFrame = self.rectangualarMasking(frame, mCoordinates, mExtents, self.maskBuffer)
        else:
            maskedFrame = frame

        self.frameHistory["masked frame"] = maskedFrame

        # Color processing the image with saturation and my custom colorProcessing function
        images = self.imageColorProcessing(maskedFrame)

        self.frameHistory["blue image"] = images[0]
        self.frameHistory["green image"] = images[1]
        self.frameHistory["red image"] = images[2]
        
        self.frameHistory["white image"] = images[3]

        # Tracking the induvidual components in an array
        refined_components = self.componentProcessing(images, frame)

        self.frameHistory["final"] = frame

        return refined_components, frame

    def imageColorProcessing(self, frame):
        # Splitting BGR image
        channels = cv2.split(frame)

        # Processing target color channel and white channel || Blurring to remove hard edges
        blueImage = cv2.GaussianBlur(channels[0], (11, 11), 0)
        greenImage = cv2.GaussianBlur(channels[1], (11, 11), 0)
        redImage = cv2.GaussianBlur(channels[2], (11, 11), 0)

        grayscaleImage = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (11, 11), 0)

        images = (blueImage, greenImage, redImage, grayscaleImage)
        
        return images 

    def componentProcessing(self, images, frame):
        # Threshholding (grayscale) white and target color channel images
        threshholdBlueMin = cv2.threshold(images[0], self.threshholdBlueMin, 255, cv2.THRESH_BINARY)[1]
        threshholdGreenMin = cv2.threshold(images[1], self.threshholdGreenMin, 255, cv2.THRESH_BINARY)[1]
        threshholdRedMin = cv2.threshold(images[2], self.threshholdRedMin, 255, cv2.THRESH_BINARY)[1]

        threshholdWhiteMin = cv2.threshold(images[3], self.threshholdBrightMin, 255, cv2.THRESH_BINARY)[1]

        threshholdBlueMax = cv2.threshold(images[0], self.threshholdBlueMax, 255, cv2.THRESH_BINARY)[1]
        threshholdGreenMax = cv2.threshold(images[1], self.threshholdGreenMax, 255, cv2.THRESH_BINARY)[1]
        threshholdRedMax = cv2.threshold(images[2], self.threshholdRedMax, 255, cv2.THRESH_BINARY)[1]

        threshholdWhiteMax = cv2.threshold(images[3], self.threshholdBrightMax, 255, cv2.THRESH_BINARY)[1]

        threshholdBlue = threshholdBlueMin - threshholdBlueMax
        threshholdGreen = threshholdGreenMin - threshholdGreenMax
        threshholdRed = threshholdRedMin - threshholdRedMax

        threshholdWhite = threshholdWhiteMin - threshholdWhiteMax

        threshholds = [threshholdBlue, threshholdGreen, threshholdRed, threshholdWhite]
        threshhold = threshholds[self.targetChannel]

        # Subtracting white threshhold to avoid tracking bright lights maxing out all BGR channels
        if not self.targetChannel == 3:
            for i, currentThreshhold in enumerate(threshholds):
                if not np.array_equiv(threshholds[self.targetChannel], currentThreshhold):
                    currentThreshhold = cv2.dilate(currentThreshhold, None, iterations = self.dialationSteps[i])
                    threshholds[i] = currentThreshhold

                    threshhold -= (currentThreshhold)

        self.frameHistory["threshholding blue"] = threshholdBlue # threshholds[0]
        self.frameHistory["threshholding green"] = threshholdGreen # threshholds[1]
        self.frameHistory["threshholding red"] = threshholdRed # threshholds[2]

        self.frameHistory["threshholding white"] = threshholds[3]

        self.frameHistory["combination"] = threshhold

        # using cv2 morphology to remove noise and small light sources (e.g. reflections)
        threshhold = cv2.morphologyEx(threshhold, cv2.MORPH_OPEN, None, iterations=self.erosionSteps)

        self.frameHistory["tracking image"] = threshhold

        # re-threshholding the processed image to purify the white output and avoid possible dark connecting islands
        threshhold = cv2.threshold(threshhold, 200, 255, cv2.THRESH_BINARY)[1] # REMOVED FOR TESTING PUROPSES
        
        # Letting OpenCV recognise the white islands left by previous operations
        connected_components_stats = cv2.connectedComponentsWithStats(threshhold, 1, cv2.CV_32S)  # Note: Adjust algorithm

        # Getting statistics about those islands
        components = connected_components_stats[2]
        total_components = connected_components_stats[0]

        refined_components = []

        for i in range(total_components - 1):
            # Rendering the boxes in the correct location
            x = components[i + 1, cv2.CC_STAT_LEFT]
            y = components[i + 1, cv2.CC_STAT_TOP]
            width = components[i + 1, cv2.CC_STAT_WIDTH]
            height = components[i + 1, cv2.CC_STAT_HEIGHT]
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 3)

            # Refining the components by properly arranging the statistics and adjusting the coordinates to the center of the track
            xAdjusted = x + width / 2
            yAdjusted = y + height / 2

            refined_components.append([i, (x, y), (width, height)])

        return refined_components

    def rectangualarMasking(self, frame, coordinates, extents, buffer):
        # Initialising mask
        mask = np.zeros(frame.shape[:2], dtype="uint8")

        # Setting up the mask with coordinates, radius, color and invert
        cv2.rectangle(mask, (int(coordinates[0] - buffer), int(coordinates[1] - buffer)), (int(coordinates[0] + extents[0] + buffer), int(coordinates[1] + extents[1] + buffer)), 255, -1)
        maskedImage = cv2.bitwise_and(frame, frame, mask = mask)

        # Drawing masked region as red circle
        cv2.rectangle(frame, (int(coordinates[0] - buffer), int(coordinates[1] - buffer)), (int(coordinates[0] + extents[0] + buffer), int(coordinates[1] + extents[1] + buffer)), 255, 2)

        return maskedImage

    def adjustSaturation(self, frame, saturation):
        # Converting the color to HSV and avoiding unit8 overflow error with float values
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype("float32")
        (h, s, v) = cv2.split(hsv)

        # Saturation value adjustment + overflow clipping
        s *= saturation
        s = np.clip(s, 0, 255)

        # Merging adjusted HSV channels
        saturatedImage = cv2.merge([h, s, v])

        # Using uni8 again for BGR color spectrum
        return cv2.cvtColor(saturatedImage.astype("uint8"), cv2.COLOR_HSV2BGR)

    def adjustContrast(self, frame, contrast = 127, brightness = 255):
        brightness = int((brightness - 0) * (255 - (-255)) / (510 - 0) + (-255))
        contrast = int((contrast - 0) * (127 - (-127)) / (254 - 0) + (-127))

        if brightness != 0:
            if brightness > 0:
                shadow = brightness
                max = 255
            else:
                shadow = 0
                max = 255 + brightness

            brightnessAlpha = (max - shadow) / 255
            brightnessGamma = shadow

            buffer = cv2.addWeighted(frame, brightnessAlpha, frame, 0, brightnessGamma)
        else:
            buffer = frame

        if contrast != 0:
            contrastAlpha = float(131 * (contrast + 127)) / (127 * (131 - contrast))
            contrastGamma = 127 * (1 - contrastAlpha)
            
            buffer = cv2.addWeighted(buffer, contrastAlpha, buffer, 0, contrastGamma)

        return buffer


if __name__ == '__main__':
    # Debugging purposes
    tracker = systemTrack(1)

    tracker.mainCycle()

    cv2.imshow('Frame', tracker.frame)
    cv2.waitKey(0)
