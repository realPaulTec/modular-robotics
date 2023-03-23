#!../.venv/bin/python3.11

from component import Component
import numpy as np
import time
import cv2
import IPutils

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

        self.useMask = True
        self.maskBuffer = 40

        self.trackComponents = []

        self.MAX_MATRIX_DIFFERENCE = 20
        self.MAX_LIFETIME_SECONDS = 4

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
        print('=== NEW =========================== NEW ===')

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
        threshold = self.processing_thresholds(thresholds, frame)

        # Letting OpenCV recognize the white islands left by previous operations
        connectedComponentsStats = cv2.connectedComponentsWithStats(threshold, 1, cv2.CV_32S)

        # Refining the components into component classes 
        componentsN = self.refine_components(connectedComponentsStats)

        # Filtering the LiDAR components, compared to their historical counterparts. 
        if len(self.trackComponents) > 0:
            self.refinedComponents = self.filter_components(componentsN)
        else:
            self.trackComponents = componentsN
            self.refinedComponents = self.trackComponents


        # ===== Rendering =========================================
        # Rendering green boxes where components were detected!
        canvasFrame = self.render_components(self.refinedComponents, canvasFrame)

        # Setting the final result to the frame history!
        self.frameHistory["final"] = canvasFrame

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
        threshold = thresholds[self.targetChannel]    

        # Subtracting white threshold to avoid tracking bright lights maxing out all BGR channels
        if not self.targetChannel == 3:
            for i, currentThreshold in enumerate(thresholds):
                if not np.array_equiv(thresholds[self.targetChannel], currentThreshold):
                    currentThreshold = cv2.dilate(currentThreshold, None, iterations = self.dilationSteps[i])
                    thresholds[i] = currentThreshold

                    threshold -= (currentThreshold)

        # using cv2 morphology to remove noise and small light sources (e.g. reflections)
        thresholdClean = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, None, iterations=self.erosionSteps)

        # re-thresholding the processed image to purify the white output and avoid possible dark connecting islands
        thresholdClean = cv2.threshold(thresholdClean, 200, 255, cv2.THRESH_BINARY)[1]

        # Showing the results of this step in the frame history.
        self.frameHistory["thresholding blue"] = thresholds[0]
        self.frameHistory["thresholding green"] = thresholds[1]
        self.frameHistory["thresholding red"] = thresholds[2]
        self.frameHistory["thresholding white"] = thresholds[3]
        self.frameHistory["combination"] = threshold
        self.frameHistory["tracking image"] = thresholdClean

        return thresholdClean
    
    def refine_components(self, componentsTRK):
        componentsN = []

        # Getting statistics about those islands
        components = componentsTRK[2]
        totalComponents = componentsTRK[0]

        # Making a dictionary of the components. TODO: Change to np.array for performance reasons.
        for i in range(totalComponents - 1):
            component = Component(
                components[i + 1, cv2.CC_STAT_LEFT],                        # x coordinate
                components[i + 1, cv2.CC_STAT_TOP],                         # y coordinate
                components[i + 1, cv2.CC_STAT_WIDTH],                       # Width
                components[i + 1, cv2.CC_STAT_HEIGHT],                      # Height
                components[i + 1, cv2.CC_STAT_AREA],                        # Area
                np.array((componentsTRK[1] == (i + 1)).astype(np.uint8))    # Geometry (matrix)
            )
                
            componentsN.append(component)

        componentsN = np.array(componentsN)

        return componentsN
    
    def filter_components(self, componentsN):
        elementList = []
        
        for component in self.trackComponents:
            # Get the index of the most similar historical component to the current component | TODO: Fix duplicates which are similar to two different historical arrays!
            simOutput = component.check_similarity_array(componentsN)
            
            # Pass the component to the next generation if it has found a child!
            if simOutput != None:
                index, similarity = simOutput

                # Actually pass it to the next generation!
                if similarity <= self.MAX_MATRIX_DIFFERENCE:
                    componentsN[int(index)].update_component(component.ID, component.generate_matrix())
                    elementList.append(componentsN[int(index)])

                    # print(f'Child found for Component: {component.ID}')
                else:
                    print(f'Dropped: {component.ID} for Similarity: {similarity}') # NOTE: The component area is massive, so it has a large impact! I HAVE to implement weight! For now the area will be dropped in the MATRIX!!!

        # Appending all new components!
        for component in componentsN:
            if component not in elementList:
                elementList.append(component)

        # Set the new updated components!
        self.trackComponents = np.array(elementList)

        print(f"There are {len(self.trackComponents)} components!")

        return self.trackComponents

    def render_components(self, components, canvasFrame):
        for component in components:
            cv2.rectangle(canvasFrame, (component.x, component.y), (component.x + component.width, component.y + component.height), (0, 255, 0), 3)

        return canvasFrame


if __name__ == '__main__':
    # Debugging purposes
    tracker = systemTrack(1)

    tracker.mainCycle()

    cv2.imshow('Frame', tracker.frame)
    cv2.waitKey(0)
