import cv2
import numpy as np


class ColorDetection:
    def __init__(self, hsv_inferior=None, hsv_superior=None, min_area=1000):
        if hsv_inferior is None:
            # H: 100, S: 150, V: 50
            # H - Hue (Matiz) | S - Saturation (Saturação) |V - Value (Valor ou Brilho)
            self.hsv_inferior = np.array([100, 150, 50], dtype=np.uint8)
        else:
            self.hsv_inferior = hsv_inferior

        if hsv_superior is None:
            # H: 130, S: 255, V: 255
            self.hsv_superior = np.array([130, 255, 255], dtype=np.uint8)
        else:
            self.hsv_superior = hsv_superior

# Alterar para detectar azul - cor da base de pouso
    def detectar_cor_azul(frame, contour):
        # Converte o frame para o espaço de cor HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        # Faixa 1 (para azuis mais claros)
        # H: 95-110, S: 120-255, V: 70-255
        lower_blue1 = np.array([95, 120, 70])
        upper_blue1 = np.array([110, 255, 255])
        mask1 = cv2.inRange(hsv_frame, lower_blue1, upper_blue1)

        # Faixa 2 (para azuis mais escuros)
        # H: 111-130, S: 150-255, V: 50-255
        lower_blue2 = np.array([111, 150, 50])
        upper_blue2 = np.array([130, 255, 255])
        mask2 = cv2.inRange(hsv_frame, lower_blue2, upper_blue2)
        
        # Combina as duas máscaras de cor para detectar ambos os tons de azul
        mask_cor_azul = mask1 + mask2

        # Cria uma máscara para a forma do contorno detectado
        mascara_contorno = np.zeros(frame.shape[:2], dtype="uint8")
        cv2.drawContours(mascara_contorno, [contour], -1, 255, -1)

        # Combina a máscara de cor com a máscara de contorno.
        mascara_combinada = cv2.bitwise_and(mask_cor_azul, mask_cor_azul, mask=mascara_contorno)

        # Se houver pixels suficientes na máscara combinada, a cor foi encontrada dentro da forma
        if cv2.countNonZero(mascara_combinada) > 50:  # 50 pixels de tolerância
            return True
        
        return False