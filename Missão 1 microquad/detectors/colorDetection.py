import cv2
import numpy as np
from config import COLOR_DETECTION_CONFIG

class ColorDetection:
    def __init__(self):
        self.lower_blue1 = COLOR_DETECTION_CONFIG["lower_blue1"]
        self.upper_blue1 = COLOR_DETECTION_CONFIG["upper_blue1"]
        self.lower_blue2 = COLOR_DETECTION_CONFIG["lower_blue2"]
        self.upper_blue2 = COLOR_DETECTION_CONFIG["upper_blue2"]
        self.pixel_threshold = COLOR_DETECTION_CONFIG["pixel_threshold"]

    def is_contour_blue(self, frame, contour):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv_frame, self.lower_blue1, self.upper_blue1)
        mask2 = cv2.inRange(hsv_frame, self.lower_blue2, self.upper_blue2)
        
        mask_cor_azul = mask1 + mask2

        mascara_contorno = np.zeros(frame.shape[:2], dtype="uint8")
        cv2.drawContours(mascara_contorno, [contour], -1, 255, -1)

        mascara_combinada = cv2.bitwise_and(mask_cor_azul, mask_cor_azul, mask=mascara_contorno)

        if cv2.countNonZero(mascara_combinada) > self.pixel_threshold:
            return True
        
        return False