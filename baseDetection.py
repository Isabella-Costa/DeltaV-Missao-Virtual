import cv2
from shapeDetection import ShapeDetection
from colorDetection import ColorDetection

class BaseDetector:
    def __init__(self):
        self.shape_detector = ShapeDetection(min_area=800) 
        self.color_detector = ColorDetection()

    def detect(self, frame):
        all_shapes = self.shape_detector.detecta_contorno(frame)

        for shape in all_shapes:
            if shape['label'] == "Circulo":
                contour = shape['contour']
                if self.color_detector.is_contour_blue(frame, contour):
                    (x, y), radius = cv2.minEnclosingCircle(contour)

                    base = { 'center': (int(x), int(y)), 'radius': int(radius), 'contour': contour}
                    
                    # Retorna os dados da primeira base válida que encontrar
                    return base

        return None