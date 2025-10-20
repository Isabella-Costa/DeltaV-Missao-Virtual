import cv2

class CameraCapture:
    def __init__(self, source=0):
        self.source = source
        self.cap = cv2.VideoCapture(self.source)

        if not self.cap.isOpened():
            raise IOError(f"Erro: Não foi possível abrir a fonte de vídeo: {self.source}")

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Fonte de vídeo aberta com sucesso. Resolução: {self.width}x{self.height}")

    def get_frame(self):
        ret, frame = self.cap.read()
        return ret, frame

    def release(self):
        print("Liberada a fonte de vídeo.")
        self.cap.release()