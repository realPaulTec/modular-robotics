#!../.venv/bin/python3.11

from component import Component
import numpy as np
import time
import cv2
import IPutils

# TODO:
# Render modulating component(s) separately. 

class systemTrack:
    def __init__(self, vcd, targetChannel=0, aqqExtents=(100, 100), saturationAdjustment=1.2, contrastAdjustment=1.02, brightnessAdjustment=0) -> None:
        # Setting up video capture device (camera)
        self.video = cv2.VideoCapture(vcd)

        if not self.video.isOpened():
            raise IOError("Cannot open camera")

        self.videoDimensions = (self.video.get(cv2.CAP_PROP_FRAME_WIDTH), self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.acquisitionExtents = aqqExtents
        self.targetChannel = targetChannel
        self.saturationAdjustment = saturationAdjustment
        self.contrastAdjustment = contrastAdjustment
        self.brightnessAdjustment = brightnessAdjustment

        self.frame = None
        self.refinedComponents = []

        self.thresholdRedMax = 255
        self.thresholdRedMin = 230

        self.thresholdGreenMax = 255
        self.thresholdGreenMin = 230

        self.thresholdBlueMax = 255
        self.thresholdBlueMin = 230

        self.thresholdBrightMax = 255
        self.thresholdBrightMin = 200

        self.dilationSteps = [6, 6, 6, 6]

        self.erosionSteps = 4

        self.useMask = False
        self.maskBuffer = 40

        self.trackComponents = []

        self.MAX_MATRIX_DIFFERENCE = 0
        self.MAX_LIFETIME_SECONDS = 4

        self.wCoordinates = 1
        self.wBounds = 1
        self.wArea = 1

        self.frequency = 10
        self.frequencyBuffer = 2
        self.FREQUENCY_SAMPLES = 4
        self.ΔTimeFactorHistory = 2.5
        self.lastFrequencyUpdate = time.time()

        self.ΔTime = 0

        self.closestFrequency = 0

        self.frameHistory = {
            "camera feed" : None,
            "color corrected" : None,
            "masked frame" : None,
            "blue image" : None,
            "green image" : None,
            "red image" : None,
            "white image" : None,
            "thresholding blue" : None,
            "thresholding green" : None,
            "thresholding red" : None,
            "thresholding white" : None,
            "combination" : None,
            "tracking image" : None,
            "final" : None
        }

    def mainCycle(self):
        # Δt No.1
        time_1 = time.time()

        # ===== Reading Camera ====================================

        stream = self.video.read()
        frame = stream[1]

        canvasFrame = np.array(frame)

        self.frameHistory["camera feed"] = frame


        # ===== Color Processing ==================================

        # Applying color correction
        frameCC = self.color_correction(frame)

        # Masking the frame based on mask settings
        frameMK, canvasFrame = self.set_mask(frameCC, canvasFrame, self.refinedComponents)

        # Turning all images into grayscale
        images = self.processing_images(frameMK)


        # ===== Applying Thresholds ===============================

        thresholds = []

        # Setting the min and max thresholds for the BGR images!  
        minThresholds = (self.thresholdBlueMin, self.thresholdGreenMin, self.thresholdRedMin, self.thresholdBrightMin)
        maxThresholds = (self.thresholdBlueMax, self.thresholdGreenMax, self.thresholdRedMax, self.thresholdBrightMax)

        # Looping trough all grayscale images and applying the correct min and max threshold!
        for i, image in enumerate(images):
            thresholdMin = cv2.threshold(image, minThresholds[i], 255, cv2.THRESH_BINARY)[1]
            thresholdMax = cv2.threshold(image, maxThresholds[i], 255, cv2.THRESH_BINARY)[1]

            thresholds.append(
                thresholdMin - thresholdMax
            )


        # ===== Component Tracking ================================

        # Tracking the individual components in an array
        finalThreshold = self.processing_thresholds(thresholds, frame)

        # Letting OpenCV recognize the white islands left by previous operations
        connectedComponentsStats = cv2.connectedComponentsWithStats(finalThreshold, 1, cv2.CV_32S)

        # Refining the components into component classes 
        newComponents = self.refine_components(connectedComponentsStats)

        # FIXME: # Filtering the LiDAR components, compared to their historical counterparts. 
        if len(self.trackComponents) > 0:
            self.refinedComponents = self.filter_components(newComponents)
        else:
            self.trackComponents = newComponents
            self.refinedComponents = self.trackComponents


        # ===== Rendering =========================================

        # Rendering green boxes where components were detected!
        canvasFrame = self.render_components(self.refinedComponents, canvasFrame)

        # Setting the final result to the frame history!
        self.frameHistory["final"] = np.array(canvasFrame)

        # Δt No.2
        self.ΔTime = time.time() - time_1
        # print(self.ΔTime)

        return frame, self.refinedComponents, self.frameHistory

    def color_correction(self, frame):
        frameCC = IPutils.adjustSaturation(frame, self.saturationAdjustment)
        # frame = IPutils.adjustContrast(frame, self.contrastAdjustment, self.brightnessAdjustment)
        self.frameHistory["color corrected"] = frameCC

        return frameCC

    def set_mask(self, frame, canvasFrame, components):
        if len(components) > 0:
            # Using first track coordinates as mask coordinates
            x, y = components[0].x, components[0].y
            coordinates = (int(x), int(y))

            # Using first track dimensions as mask radius
            (width, height) = components[0].width, components[0].height
            extents = (int(width), int(height))
        else:
            # Using video center as mask coordinates
            coordinates = (int((self.videoDimensions[0] / 2) - (self.acquisitionExtents[0] / 2)), int((self.videoDimensions[1] / 2) - (self.acquisitionExtents[1] / 2)))
            extents = self.acquisitionExtents

        # Applying mask with my masking function
        if self.useMask == True:
            maskedFrame, canvasFrame = IPutils.rectangular_masking(frame, canvasFrame, coordinates, extents, self.maskBuffer)
        else:
            maskedFrame = frame

        # Show the results of this step in the frame history!
        self.frameHistory["masked frame"] = maskedFrame

        return maskedFrame, canvasFrame

    def processing_images(self, frame):
        # Splitting BGR image
        channels = cv2.split(frame)

        # Processing target color channel and white channel || Blurring to remove hard edges
        blueImage = cv2.GaussianBlur(channels[0], (11, 11), 0)
        greenImage = cv2.GaussianBlur(channels[1], (11, 11), 0)
        redImage = cv2.GaussianBlur(channels[2], (11, 11), 0)

        grayscaleImage = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (11, 11), 0)

        images = (blueImage, greenImage, redImage, grayscaleImage)
        
        # Show the results of this step in the frame history!
        self.frameHistory["blue image"] = images[0]
        self.frameHistory["green image"] = images[1]
        self.frameHistory["red image"] = images[2]
        
        self.frameHistory["white image"] = images[3]

        return images 

    def processing_thresholds(self, thresholds, frame):
        # Get the thresholds
        threshold = np.array(thresholds[self.targetChannel])

        # Subtracting white threshold to avoid tracking bright lights maxing out all BGR channels
        if not self.targetChannel == 3:
            for i, currentThreshold in enumerate(thresholds):
                if not np.array_equiv(thresholds[self.targetChannel], currentThreshold):
                    currentThreshold = cv2.dilate(currentThreshold, None, iterations = self.dilationSteps[i])
                    thresholds[i] = np.array(currentThreshold)

                    threshold -= (currentThreshold)

        # using cv2 morphology to remove noise and small light sources (e.g. reflections)
        thresholdClean = np.array(cv2.morphologyEx(np.array(threshold), cv2.MORPH_OPEN, None, iterations=self.erosionSteps))

        # re-thresholding the processed image to purify the white output and avoid possible dark connecting islands | 
        thresholdClean = cv2.threshold(thresholdClean, 200, 255, cv2.THRESH_BINARY)[1]
        # thresholdClean[thresholdClean > 0] = 1

        # Showing the results of this step in the frame history.
        self.frameHistory["thresholding blue"] = np.array(thresholds[0])
        self.frameHistory["thresholding green"] = np.array(thresholds[1])
        self.frameHistory["thresholding red"] = np.array(thresholds[2])
        self.frameHistory["thresholding white"] = np.array(thresholds[3])
        self.frameHistory["combination"] = np.array(threshold)
        self.frameHistory["tracking image"] = np.array(thresholdClean)

        return np.array(thresholdClean)
    
    def refine_components(self, componentsTRK):
        newComponents = []

        # Getting statistics about those islands
        components = componentsTRK[2]
        totalComponents = componentsTRK[0]

        # Making a dictionary of the components.
        for i in range(totalComponents - 1):
            component = Component(
                components[i + 1, cv2.CC_STAT_LEFT],                        # x coordinate
                components[i + 1, cv2.CC_STAT_TOP],                         # y coordinate
                components[i + 1, cv2.CC_STAT_WIDTH],                       # Width
                components[i + 1, cv2.CC_STAT_HEIGHT],                      # Height
                components[i + 1, cv2.CC_STAT_AREA],                        # Area
                np.array((componentsTRK[1] == (i + 1)).astype(np.uint8))    # Geometry (matrix)
            )
                
            newComponents.append(component)

        return newComponents
    
    def filter_components(self, newComponents):
        # Convert new components to numpy array for calculations
        newComponents = np.array(newComponents)

        # Unify similar components from the current cycle
        for component in newComponents:
            # Get index and difference from the closest component to the current one
            index, difference = component.check_similarity_array(newComponents, self.wCoordinates, self.wArea, self.wBounds)

            # Check for (and delete) duplicate component
            if index != None and difference != None and difference <= self.MAX_MATRIX_DIFFERENCE:
                np.delete(newComponents, int(index), 0)


        # Trying to match old components to a new child.
        for component in self.trackComponents:
            index, difference = component.check_similarity_array(newComponents, self.wCoordinates, self.wArea, self.wBounds)

            # Update the component if a new child is found withing the MAX_MATRIX_DIFFERENCE!
            if index != None and difference != None and difference <= self.MAX_MATRIX_DIFFERENCE:
                # Append the last update before the component was reacquired  
                if (time.time() - component.lastUpdate) > (self.ΔTimeFactorHistory * self.ΔTime):
                    component.history.append(time.time() - component.lastUpdate)

                newComponents[int(index)].update_component(component.ID, difference, history = component.history)
            
            # Keep the component as long as its extended lifetime permits.
            elif time.time() - component.lastUpdate < self.MAX_LIFETIME_SECONDS:
                newComponents = np.append(newComponents, component)

        frequencies = []

        for component in newComponents:
            # Generate IDs for new components which did not find a parent.
            if component.ID == None:
                component.generate_id()

            # Calculate the frequency of the component
            if len(component.history) >= self.FREQUENCY_SAMPLES:
                history = np.array(component.history)
                median = np.median(history)

                frequencies.append(round(1 / (median * 2), 2))
                component.history = []

        # Get the frequency which is the closest match to the desired one.
        if len(frequencies) > 0:
            frequencies = np.array(frequencies)
            index = (np.abs(frequencies - self.frequency)).argmin()

            # Check if the acquired frequency is within the buffer zone!
            if abs(frequencies[index] - self.frequency) <= self.frequencyBuffer:
                self.closestFrequency = frequencies[index]
                self.lastFrequencyUpdate = time.time()

        # Set it to zero if the frequency was lost!
        elif self.closestFrequency > 0 and (time.time() - self.lastFrequencyUpdate) > (2 * self.FREQUENCY_SAMPLES / self.closestFrequency):
            self.closestFrequency = 0.00

        # Processed new components are now the historical ones for the next generation!
        self.trackComponents = np.array(newComponents)
        return self.trackComponents

    def render_components(self, components, canvasFrame):
        # Render all components rectangles with a unique color based on their ID. TODO: Render components with the correct frequency separately!
        for component in components:
            component.color = IPutils.get_color_from_id(component.ID)

            cv2.rectangle(canvasFrame, (component.x, component.y), (component.x + component.width, component.y + component.height), component.color, 3)

        return canvasFrame


if __name__ == '__main__':
    # Debugging purposes
    tracker = systemTrack(1)

    tracker.mainCycle()

    cv2.imshow('Frame', tracker.frame)
    cv2.waitKey(0)
