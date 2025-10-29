import cv2
import time
from threading import Lock, Thread


class Camera:
    """
    Classe para gerenciar a camera em uma thread separada,
    fornecendo frames brutos para processamento no loop principal da aplicação.
    """

    H_PIXELS = 640
    H_FOV_DEG = 62.2

    def __init__(self, resolution=(640, 480)):
        self.camera = cv2.VideoCapture("rtsp://localhost:8554/mystream")
        self.frame = None
        self.lock = Lock()
        self.running = False
        self.thread = Thread(target=self._capture_loop)

    def start(self):
        """Inicia a captura de frames da câmera em background."""
        print("Iniciando a câmera...")
        self.running = True
        self.thread.start()
        # Aguarda um tempo para a câmera estabilizar
        time.sleep(2.0)
        print("Câmera pronta.")

    def stop(self):
        """Para a thread de captura e libera a câmera."""
        if not self.running:
            return
        print("Parando a câmera...")
        self.running = False
        self.thread.join() # Espera a thread terminar
        print("Câmera parada.")

    def _capture_loop(self):
        """Loop executado em uma thread que continuamente captura frames."""
        while self.running:
            # Captura o frame como um array NumPy
            ret, array = self.camera.read()
            if not ret:
                continue
            # Bloqueia o acesso para uma atualização segura do frame
            with self.lock:
                self.frame = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)

    def read(self):
        """
        Retorna o frame mais recente capturado pela câmera.
        Retorna uma tupla (sucesso, frame), similar à interface do OpenCV.
        """
        with self.lock:
            if self.frame is None:
                return False, None
            # Retorna uma cópia para evitar problemas de concorrência entre threads
            return True, self.frame.copy()
