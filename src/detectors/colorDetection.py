import cv2
import numpy as np

class ColorDetection:
    """
    Classe para detectar se um contorno específico em uma imagem contém 
    uma cor (neste caso, azul) acima de um certo limiar de pixels.

    Atributos:
        lower_blue1 (np.array): Limite inferior da primeira faixa HSV para azul.
        upper_blue1 (np.array): Limite superior da primeira faixa HSV para azul.
        lower_blue2 (np.array): Limite inferior da segunda faixa HSV para azul.
        upper_blue2 (np.array): Limite superior da segunda faixa HSV para azul.
        pixel_threshold (int): Número mínimo de pixels de cor para que um 
                               contorno seja considerado positivo.
    """
    def __init__(self):
        """
        Inicializa o detector de cor com faixas HSV predefinidas para 
        a cor azul e um limiar de pixels.
        """
        # Faixa 1 (para azuis mais claros)
        self.lower_blue1 = np.array([95, 120, 70])
        self.upper_blue1 = np.array([110, 255, 255])
        
        # Faixa 2 (para azuis mais escuros)
        self.lower_blue2 = np.array([111, 150, 50])
        self.upper_blue2 = np.array([130, 255, 255])
        
        # Tolerância de pixels
        self.pixel_threshold = 50

    def is_contour_blue(self, frame, contour):
        """
        Verifica se um determinado contorno contém pixels azuis suficientes.

        Args:
            frame (np.ndarray): O frame/imagem original (em BGR) 
                                onde o contorno foi detectado.
            contour (np.ndarray): O contorno específico a ser verificado.

        Returns:
            bool: True se o número de pixels azuis dentro do contorno 
                  exceder o 'pixel_threshold', False caso contrário.
        """
        # 1. Converter o frame para o espaço de cores HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 2. Criar máscaras para as duas faixas de azul
        mask1 = cv2.inRange(hsv_frame, self.lower_blue1, self.upper_blue1)
        mask2 = cv2.inRange(hsv_frame, self.lower_blue2, self.upper_blue2)
        
        # 3. Combinar as duas máscaras de azul
        # (cv2.bitwise_or(mask1, mask2) também funcionaria)
        mask_cor_azul = mask1 + mask2

        # 4. Criar uma máscara preta vazia com o tamanho do frame
        mascara_contorno = np.zeros(frame.shape[:2], dtype="uint8")
        
        # 5. Desenhar (preencher) o contorno fornecido na máscara vazia
        cv2.drawContours(mascara_contorno, [contour], -1, 255, -1)

        # 6. Encontrar a interseção (bitwise AND) entre a máscara de cor e a máscara do contorno.
        #    Isso isola apenas os pixels azuis QUE ESTÃO DENTRO do contorno.
        mascara_combinada = cv2.bitwise_and(mask_cor_azul, mask_cor_azul, mask=mascara_contorno)

        # 7. Contar os pixels não-zero (brancos) na máscara resultante
        total_pixels_azuis = cv2.countNonZero(mascara_combinada)

        # 8. Compara com o limiar
        if total_pixels_azuis > self.pixel_threshold:
            return True
        
        return False
