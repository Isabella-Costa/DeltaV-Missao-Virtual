import cv2
import time

from cameraCapture import CameraCapture
from shapeDetection import ShapeDetection
from baseDetection import BaseDetector 

def main():
    ALVO_SHAPE = "Casa"

    try:
        camera = CameraCapture(source=1) 
    except IOError as e:
        print(e)
        return

    # Inicializa os dois detectores
    shape_detector = ShapeDetection(min_area=800)
    base_detector = BaseDetector() 

    print(f"Iniciando modo de detecção total. Alvo: {ALVO_SHAPE}. Pressione 'q' para sair.")

    while True:
        ret, frame = camera.get_frame()
        if not ret:
            break

        # DETECÇÃO DE FORMAS (ALVO)
        shapes_detectados = shape_detector.detecta_contorno(frame)
        
        # DETECÇÃO DA BASE 
        base_data = base_detector.detect(frame) # Assumindo que o método em BaseDetector é 'detect'

        # VISUALIZAÇÃO 
        frame_com_desenho = frame.copy()

        # Desenha todas as formas encontradas
        frame_com_desenho = ShapeDetection.draw_contorno(frame_com_desenho, shapes_detectados)

        # Desenha a base, se encontrada
        if base_data is not None:
            center = base_data['center']
            radius = base_data['radius']
            cv2.circle(frame_com_desenho, center, radius, (255, 0, 0), 3)
            cv2.circle(frame_com_desenho, center, 5, (0, 0, 255), -1)
            cv2.putText(frame_com_desenho, "BASE", (center[0] - 30, center[1] - radius - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # Mostra o resultado final com tudo desenhado
        cv2.imshow("Visao do Drone - Modo de Teste Total", frame_com_desenho)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()
    print("Teste encerrado pelo operador.")

if __name__ == "__main__":
    main()