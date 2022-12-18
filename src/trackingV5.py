import numpy as np
import cv2

# Roadmap:
# Proper GUI

class systemTrack:
    def __init__(self, vcd, targetChannel = 0, aqqRadius = 100, saturationAdjustment = 1.2, contrastAdjustment = 1.02, brightnessAdjustment = 0) -> None:
        # Setting up video capture device (camera)
        self.video = cv2.VideoCapture(vcd)
        self.videoDimensions = (self.video.get(cv2.CAP_PROP_FRAME_WIDTH), self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.acquisitionRadius = aqqRadius
        self.targetChannel = targetChannel
        self.saturationAdjustment = saturationAdjustment
        self.contrastAdjustment = contrastAdjustment
        self.brightnessAdjustment = brightnessAdjustment

        self.frame = None
        self.refined_components = []

    def mainCycle(self):
        # Reading the frame displated by the system camera
        stream = self.video.read()
        self.frame = stream[1]

        if len(self.refined_components) > 0:
            # Using first track coordinates as mask coordinates
            (x, y) = self.refined_components[0][1]
            coordinates = (int(x), int(y))
            
            # Using first track dimensions as mask radius
            (width, height) = self.refined_components[0][2]
            radius = int((width + height) / 2)
        else:
            # Using video center as mask coordinates 
            coordinates = (int(self.videoDimensions[0] / 2), int(self.videoDimensions[1] / 2))
            
            # Using standard acquisition radius as mask radius
            radius = self.acquisitionRadius
        
        self.refined_components, self.frame, out = self.trackCycle(self.frame, radius, coordinates)

        return self.refined_components, self.frame, out 

    def trackCycle(self, frame, mRadius, mCoordinates):
        # Stage one color processing
        self.frame = self.adjustSaturation(self.frame, self.saturationAdjustment)
        self.frame = self.adjustContrast(self.frame, self.contrastAdjustment, self.brightnessAdjustment)
        
        # Applying mask with my own function
        maskedFrame = self.circualarMasking(self.frame, mCoordinates, mRadius)

        # Color processing the image with saturation and my custom colorProcessing function
        colorProcessedImage, whiteImage = self.imageColorProcessing(maskedFrame, self.targetChannel)

         # Tracking the induvidual components in an array
        refined_components, out = self.componentProcessing(colorProcessedImage, whiteImage)
        
        return refined_components, self.frame, out

    def simpleUI(self, frame):
        # Showing the frame with all tracks
        cv2.imshow('frame', frame)
    
        # Continuing Program
        cv2.waitKey(1)
    
    def imageColorProcessing(self, frame, colorChannel):
        # Splitting BGR image
        channels = cv2.split(frame)
        
        # Processing target color channel and white channel || Blurring to remove hard edges
        blurredImage = cv2.GaussianBlur(channels[colorChannel], (11, 11), 0)
        grayscaleImage = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (11, 11), 0)
        return blurredImage, grayscaleImage

    def componentProcessing(self, imageToProcess, whiteImage):
        # Threshholding (grayscale) white and target color channel images
        threshhold = cv2.threshold(imageToProcess, 250, 255, cv2.THRESH_BINARY)[1]
        threshholdWhite = cv2.threshold(whiteImage, 200, 255, cv2.THRESH_BINARY)[1]
        
        # Subtracting white threshhold to avoid tracking bright lights maxing out all BGR channels 
        threshhold -= threshholdWhite
        
        # using cv2 morphology to remove noise and small light sources (e.g. reflections)
        threshhold = cv2.dilate(threshhold, None, iterations = 2)
        
        threshhold = cv2.morphologyEx(threshhold, cv2.MORPH_OPEN, None, iterations = 2)
        threshhold = cv2.dilate(threshhold, None, iterations = 20)
        threshhold = cv2.erode(threshhold, None, iterations = 10)
        
        threshholdDisplay = threshhold ## Testing
        
        # re-threshholding the processed image to purify the white output and avoid possible dark connecting islands
        threshhold = cv2.threshold(threshhold, 200, 255, cv2.THRESH_BINARY)[1]
        
        # Letting OpenCV recognise the white islands left by previous operations
        connected_components_stats = cv2.connectedComponentsWithStats(threshhold, 1, cv2.CV_32S) ## Note: Adjust algorithm
        
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
            cv2.rectangle(self.frame, (x, y), (x + width, y + height), (0, 255, 0), 3)
            
            # Refining the components by properly arranging the statistics and adjusting the coordinates to the center of the track
            xAdjusted = x + width / 2
            yAdjusted = y + height / 2
            
            refined_components.append([i, ( xAdjusted, yAdjusted), (width, height)])
        
        return refined_components, threshholdDisplay

    def componentProcessingCLAHE(self, imageToProcess, whiteImage):
        #CLAHE operations
        clahe = cv2.createCLAHE()    
        cv2.cvtColor(clahe.apply(cv2.cvtColor(imageToProcess, cv2.COLOR_BGR2LAB)), cv2.COLOR_LAB2BGR)
        cv2.cvtColor(clahe.apply(cv2.cvtColor(whiteImage, cv2.COLOR_BGR2LAB)), cv2.COLOR_LAB2BGR)
        # Threshholding (grayscale) white and target color channel images
        threshhold = cv2.threshold(imageToProcess, 250, 255, cv2.THRESH_BINARY)[1]
        threshholdWhite = cv2.threshold(whiteImage, 200, 255, cv2.THRESH_BINARY)[1]
        
        # Subtracting white threshhold to avoid tracking bright lights maxing out all BGR channels 
        threshhold -= threshholdWhite
        
        # using cv2 morphology to remove noise and small light sources (e.g. reflections)
        threshhold = cv2.dilate(threshhold, None, iterations = 2)
        
        threshhold = cv2.morphologyEx(threshhold, cv2.MORPH_OPEN, None, iterations = 2)
        threshhold = cv2.dilate(threshhold, None, iterations = 20)
        threshhold = cv2.erode(threshhold, None, iterations = 10)
        
        threshholdDisplay = threshhold ## Testing
        
        # re-threshholding the processed image to purify the white output and avoid possible dark connecting islands
        threshhold = cv2.threshold(threshhold, 200, 255, cv2.THRESH_BINARY)[1]
        
        # Letting OpenCV recognise the white islands left by previous operations
        connected_components_stats = cv2.connectedComponentsWithStats(threshhold, 1, cv2.CV_32S) ## Note: Adjust algorithm
        
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
            
            refined_components.append([i, ( xAdjusted, yAdjusted), (width, height)])

        return refined_components, threshholdDisplay

    def circualarMasking(self, frame, coordinates, radius):
        # Initialising mask 
        mask = np.zeros(frame.shape[:2], dtype = "uint8")
        
        # Setting up the mask with coordinates, radius, color and invert
        cv2.circle(mask, coordinates, radius, 255, -1)    
        maskedImage = cv2.bitwise_and(frame, frame, mask = mask)

        # Drawing masked region as red circle
        cv2.circle(self.frame, coordinates, radius, (0, 0, 255), 4)

        return maskedImage

    def adjustSaturation(self, frame, saturation):
        # Converting the color to HSV and avoiding unit8 overflow error with float values
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype("float32")
        (h, s, v) = cv2.split(hsv)
        
        # Saturation value adjustment + overflow clipping
        s *= saturation
        s = np.clip(s, 0, 255)
        
        #Merging adjusted HSV channels
        saturatedImage = cv2.merge([h, s, v])

        # Using uni8 again for BGR color spectrum
        return cv2.cvtColor(saturatedImage.astype("uint8"), cv2.COLOR_HSV2BGR)

    def adjustContrast(self, frame, contrast, brightness):
        alpha = contrast
        beta = brightness
        
        processedImage = cv2.convertScaleAbs(frame, alpha = alpha, beta = beta)
        return processedImage
