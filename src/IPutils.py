import numpy as np
import cv2

def adjustSaturation(frameIn, saturation):
    # Converting the color to HSV and avoiding unit8 overflow error with float values
    hsv = cv2.cvtColor(frameIn, cv2.COLOR_BGR2HSV).astype("float32")
    (h, s, v) = cv2.split(hsv)

    # Saturation value adjustment + overflow clipping
    s *= saturation
    s = np.clip(s, 0, 255)

    # Merging adjusted HSV channels
    saturatedImage = cv2.merge([h, s, v])

    # Using uni8 again for BGR color spectrum
    return cv2.cvtColor(saturatedImage.astype("uint8"), cv2.COLOR_HSV2BGR)

def adjustContrast(frame, contrast = 127, brightness = 255):
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

def rectangular_masking(frame, canvasFrame, coordinates, extents, buffer):
    # Initializing mask
    mask = np.zeros(frame.shape[:2], dtype="uint8")

    # Setting up the mask with coordinates, radius, color and invert
    cv2.rectangle(mask, (int(coordinates[0] - buffer), int(coordinates[1] - buffer)), (int(coordinates[0] + extents[0] + buffer), int(coordinates[1] + extents[1] + buffer)), 255, -1)
    maskedImage = cv2.bitwise_and(frame, frame, mask = mask)

    # Drawing masked region as red circle
    cv2.rectangle(canvasFrame, (int(coordinates[0] - buffer), int(coordinates[1] - buffer)), (int(coordinates[0] + extents[0] + buffer), int(coordinates[1] + extents[1] + buffer)), 255, 2)

    return maskedImage, canvasFrame