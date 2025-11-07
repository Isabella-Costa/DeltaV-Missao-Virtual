import cv2
from .colorDetection import ColorDetection


class BaseDetector:
    """
    A classe BaseDetector é responsável por encontrar a BASE azul
    dentro da imagem analisada.

    O que é a "BASE"?
    -----------------
    É um círculo azul que serve como ponto de referência fixo no sistema.
    Ele é usado com o objetivo, por exemplo, do robô ou o sistema de visão saiba
    onde está o ponto inicial ou o alvo fixo no ambiente.

    Funcionamento:
    ---------------
    1. Analisa todas as formas detectadas (tais como círculos e quadrados).
    2. Verifica se tem algum círculo que tenha a cor azul em sua maioria.
    3. Caso encontre, retorna as informações da BASE (centro, raio e contorno).
    """
    def __init__(self):
        """
        Inicia o detector da base.

        Como funciona essa função:
        ----------------------
        - É criado um objeto chamado `ColorDetection`,
          que vai conter os métodos de verificação da cor de um contorno.
        """
        self.color_detector = ColorDetection()

    def detect(self, all_shapes, frame):
         """
        Faz a busca e identificação da BASE azul no frame.

        Parâmetros:
        ------------
        all_shapes : list
            Faz uma listagem de todas as formas detectadas na imagem.
            Cada forma é um dicionário com informações como:
              - 'label': nome da forma (ex.: 'Circulo', 'Quadrado')
              - 'contour': borda da forma
              - 'area': tamanho aproximado
              - 'center': posição central da forma
        frame : ndarray
            Imagem completa atual (frame) em que será feita a busca pela base.

        Retorna:
        --------
        dict ou None
            Se encontrar o círculo azul, retorna um dicionário com:
                - 'center': coordenadas do centro (x, y)
                - 'radius': tamanho do raio do círculo
                - 'contour': o contorno completo da forma
            Caso não encontre nenhuma base azul, retorna `None`.
        """

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
