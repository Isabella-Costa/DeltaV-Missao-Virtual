import cv2
import numpy as np
from config import FILTRO_KALMAN

# Incializando o filtro

def inicializar_kalman():
    kf = cv2.KalmanFilter(4, 2)


    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], np.float32)


    kf.measurementMatrix = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ], np.float32)

    kf.processNoiseCov = np.eye(4, dtype=np.float32) * FILTRO_KALMAN["uncertainty_magnitude"]
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * FILTRO_KALMAN["noise_magnitude"]

    return kf