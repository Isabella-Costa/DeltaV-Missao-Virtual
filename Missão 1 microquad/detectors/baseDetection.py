import cv2
from .colorDetection import ColorDetection


class BaseDetector:
    def __init__(self):
        self.color_detector = ColorDetection()

    def detect(self, all_shapes, frame):
        if all_shapes is None:
            return None

        for shape in all_shapes:
            # Procura pelo círculo que foi detectado 
            if shape['label'] == "Circulo":
                contour = shape['contour']
                
                # verificar se é azul
                if self.color_detector.is_contour_blue(frame, contour):
                    
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    base = { 'center': (int(x), int(y)), 'radius': int(radius), 'contour': contour}
                    
                    return base

        # Se não encontrou nenhum círculo azul
        return None