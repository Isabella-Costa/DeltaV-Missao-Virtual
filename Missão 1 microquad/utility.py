import cv2
import numpy as np
from config import ANGULACAO, FILTRAGEM_CONTORNOS


#Usando produto escalar aproximação por meio de ângulos internos do quadrado aproximado
def calcular_angulo(p1, p2, p3):
    v1, v2 = p1 - p2, p3 - p2
    cosang = np.dot(v1, v2) / (np.linalg.norm(v1)*np.linalg.norm(v2))
    return np.degrees(np.arccos(cosang))

#Válidação do quadrado
def validar_quadrado(approx, area): 
    if len(approx) != 4 or area > FILTRAGEM_CONTORNOS["min_area"]:
        return False
    angulos = [
        calcular_angulo(approx[j][0], approx[j+1]%4[0], approx[j+2]%4[0]) # Usando o ângulo encontrado pelo produto escalar para aproximar
        for j in range(4) # 4 lados
    ]

    if not all((ANGULACAO["lower_limit"]) <= ang <= ANGULACAO["upper_limit"] for ang in angulos):
        return False
    rect = cv2.minAreaRect(approx)
    (w, h) = rect[1]

    if h == 0:
        return False
    
    aspect_ratio = max(w, h) / min(w, h)
    return FILTRAGEM_CONTORNOS["min_aspect_ratio"] <= aspect_ratio <= FILTRAGEM_CONTORNOS["max_aspect ratio"]
 