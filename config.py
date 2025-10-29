import cv2

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

ANGULACAO_HEXAGONO_REG = {
    "lower_limit": 110, #Tolerância de 10°. 120 - 10
    "upper_limit": 130, #Tolerância de 10°. 120° + 10°
}

# Parâmetros do filtro de Kalman
FILTRO_KALMAN = {
    "uncertainty_magnitude": 0.03,
    "noise_magnitude": 50,
}

SOLIDEZ = {
    "star_min": 0.45,
    "star_max": 0.6,
    "cross_min": 0.7,
    "cross_max": 0.9,
    "circularidade_min": 0.85
}
# Parâmetros de Janela e Visualização
JANELA_CONFIG = {
    "LARGURA_RESIZE": 800,
    "WINDOW_NAME_NORMAL_PREFIX": "Visao do Drone - Procurando por",
    "WINDOW_NAME_CANNY": "Debug Canny",
    "WINDOW_NAME_CLAHE": "Debug CLAHE"
}