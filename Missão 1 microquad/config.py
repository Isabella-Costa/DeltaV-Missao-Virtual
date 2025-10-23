import cv2
import numpy as np 

# Parâmetros de pré-processamento
PRE_PROCESSAMENTO = {
    "clahe_cliplimit": 2.0,
    "clahe_grid_size":  (8, 8),
    "gaussian_blur_ksize": (7, 7),
}

# Parâmetros de filtragem de contornos
FILTRAGEM_CONTORNOS = {
    "contour_epsilon": 0.02,
    "min_area": 500,
    "min_aspect_ratio": 0.95,
    "max_aspect_ratio": 1.2
}

# Parâmetros de desenho
DESENHO = {
    "font": cv2.FONT_HERSHEY_SIMPLEX,
    "cor_quadrado": (0,255,0),
    "cor_centro": (0,0,255),
}

# Parâmetros de angulação
ANGULACAO = {
    "lower_limit": 80,
    "upper_limit": 100,
}

# Parâmetros de angulação do pentagono regular
ANGULACAO_PENTAGONO_REG = {
    "lower_limit": 100, #Tolerância de 8º. 108° - 8°
    "upper_limit": 116, #Tolerância de 8°. 116° - 8°
}

# Parâmetros do filtro de Kalman
FILTRO_KALMAN = {
    "uncertainty_magnitude": 0.03,
    "noise_magnitude": 50,
}

SOLIDEZ = {
    "solidez_star": 0.6,
    "solidez_cross": 0.7,
    "solidez_circle": 0.85,
    "circularidade_min": 0.85
}
