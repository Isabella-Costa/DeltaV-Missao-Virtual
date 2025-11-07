import cv2
import numpy as np
from src.config import FILTRO_KALMAN

class Tracker:
"""
    Classe responsável por realizar o rastreamento de um alvo utilizando um Filtro de Kalman.

    O Filtro de Kalman é uma ótima forma de estimar e corrigir a posição da figura-alvo quando
    em movimento, mesmo na presença de ruído ou perda temporária de detecção.

    A classe trabalha com dois estados principais:
        - Correção: quando o alvo é detectado, o filtro é ajustado com base na medição.
        - Predição: quando o alvo não é detectado, o filtro prevê a próxima posição provável.

    Atributos (ou argumentos)
    ---------
    kf : cv2.KalmanFilter
        Objeto do filtro de Kalman configurado.
    ativo : bool
        Indica se o rastreamento já foi inicializado com uma medição válida.
    prediction : tuple[int, int] | None
        Última posição prevista do alvo (x, y), ou None caso o filtro ainda não esteja ativo.
    """
    def init(self):
         """
        Inicializa o rastreamento para detectar o alvo, configurando o filtro de Kalman e variáveis de controle.
        """
        self.kf = self._incializar_kalman()
        self.ativo = False
        self.prediction = None
        
        """
        Inicializa e retorna um objeto KalmanFilter configurado.

        O filtro é definido com:
            - 4 estados: posição (x, y) e velocidade (vx, vy)
            - 2 medições: coordenadas (x, y)
            - Matrizes de transição e medição lineares
            - Covariâncias ajustadas conforme o dicionário FILTRO_KALMAN

        Retorna
        -------
        cv2.KalmanFilter
            Instância do filtro de Kalman configurado.
        """
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

        Parâmetros
         ----------
        target_found : bool
            Indica se o alvo foi detectado no frame atual.
        center : tuple[int, int]
            Coordenadas (x, y) do centro do alvo detectado.

        Retorna 
        -------
        tuple[int, int] | None
            Retorna a posição prevista (x, y) do alvo, ou None se o filtro ainda não estiver ativo.
        
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
