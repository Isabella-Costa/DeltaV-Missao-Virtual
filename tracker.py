import cv2
import numpy as np
from config import FILTRO_KALMAN

class Tracker:
    def init(self):
        self.kf = self._incializar_kalman()
        self.ativo = False
        self.prediction = None

    def _incializar_kalman(self):
        """Inicializa e retorna um objeto KalmanFilter configurado."""
        kf = cv2.KalmanFilter(4, 2)
        kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        kf.processNoiseCov = np.eye(4, dtype=np.float32) * FILTRO_KALMAN.get("uncertainty_magnitude", 1e-4)
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * FILTRO_KALMAN.get("noise_magnitude", 1e-1)
        return kf

    def update(self, target_found, center):
        """
        Atualiza o filtro Kalman.
        Corrige se um alvo for encontrado, senão, apenas prevê.
        """
        if target_found:
            # Converte o centro (tupla) para o formato do Kalman
            measurement = np.array([center[0], center[1]], np.float32)
            self.kf.correct(measurement)
            self.ativo = True

        if self.ativo:
            prediction_array = self.kf.predict()
            self.prediction = (int(prediction_array[0][0]), int(prediction_array[1][0]))
            return self.prediction

        return None