import cv2
import numpy as np

# Parâmetros de pré-processamento
PRE_PROCESSAMENTO = {
    "clahe_cliplimit": 2.0,
    "clahe_grid_size":  (8, 8),
    "gaussian_blur_ksize": (7, 7),
    "morph_kernel_close": (3, 3) #Faixas recomendadas: (3, 3) ou (5, 5) são boas
}

# Parâmetros de filtragem de contornos
FILTRAGEM_CONTORNOS = {
    "contour_epsilon": 0.009,
    "min_area": 500,
    "min_aspect_ratio": 0.95,
    "max_aspect_ratio": 1.2
}

# Parâmetros de desenho
DESENHO = {
    "font": cv2.FONT_HERSHEY_SIMPLEX,
    "font_scale": 0.7,
    "font_thickness": 2,
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
# Parâmetros de detecção do azul
COLOR_DETECTION_CONFIG = {
    "lower_blue1": np.array([95, 120, 70]),
    "upper_blue1": np.array([110, 255, 255]),
    "lower_blue2": np.array([111, 150, 50]),
    "upper_blue2": np.array([130, 255, 255]),
    "pixel_threshold": 50
}
# Mapeamento de cores por forma
COLOR_MAP = {
    "Quadrilatero": (255, 0, 0), # Azul
    "Triangulo": (0, 0, 255),    # Vermelho
    "Pentagono": (0, 255, 0),    # Verde
    "Casa": (255, 0, 255),       # Magenta
    "Cruz": (128, 0, 128),     # Roxo
    "Estrela": (0, 255, 255),    # Amarelo
    "Circulo": (0, 128, 128),    # Azul-petróleo
    "Hexagono": (255, 165, 0),   # Laranja
    "BASE": (0, 0, 255)        # Vermelho para o texto "BASE"
}

SOLIDEZ = {
    "star_min": 0.45,
    "star_max": 0.62,
    "cross_min": 0.6,
    "cross_max": 0.7,
    "circularidade_min": 0.85
}
# Parâmetros de Janela e Visualização
JANELA_CONFIG = {
    "LARGURA_RESIZE": 800,
    "WINDOW_NAME_NORMAL_PREFIX": "Visao do Drone - Procurando por",
    "WINDOW_NAME_CANNY": "Debug Canny",
    "WINDOW_NAME_CLAHE": "Debug CLAHE"
}