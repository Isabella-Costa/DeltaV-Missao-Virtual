import cv2
import numpy as np

class ColorDetection:
    def init(self):
        # Faixa 1 (para azuis mais claros)
        self.lower_blue1 = np.array([95, 120, 70])
        self.upper_blue1 = np.array([110, 255, 255])

        #Faixa 2 (para azuis mais escuros)
        self.lower_blue2 = np.array([111, 150, 50])
        self.upper_blue2 = np.array([130, 255, 255])

        # Tolerância de pixels
        self.pixel_threshold = 50

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