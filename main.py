import cv2
import numpy as np
from dronekit import connect

from controle import armar_drone_simplificado, decolar_drone_simplificado
from controle import pousar_drone_simplificado


# (Assumindo que os imports locais estão corretos)
from detectors.shapeDetection import ShapeDetector, incializar_kalman
from detectors.baseDetection import BaseDetector
from config import JANELA_CONFIG
from camera_sim import Camera


def main():

    drone = connect("udp:127.0.0.1:14550", wait_ready = True)

    # CONFIGURAÇÃO DO ALVO 
    ALVO_SHAPE = "Quadrado" 

    try:
        cam = Camera()
    except IOError as e:
        print(e)
        return

    #  Inicializa os detectores e o Kalman 
    detector = ShapeDetector()       
    base_detector = BaseDetector() 
    kalman_filter = incializar_kalman()
    kalman_ativo = False
    predicted_center = None # Inicializa a variável]
    cam.start() 
    armar_drone_simplificado(drone)
    decolar_drone_simplificado(drone, 5)
    print(f"Iniciando modo de detecção. Alvo: {ALVO_SHAPE}. Pressione 'q' para sair.")

    while True:
        # Frame original
        ret, frame_original = cam.read() 
        if not ret:
            print("Frame não capturado ou vídeo terminou.")
            break
        
        # REDIMENSIONA O FRAME 
        #frame = cv2.resize(frame_original, dim, interpolation=cv2.INTER_AREA)
        
        # DETECÇÃO
        todos_shapes_detectados, bordas_canny, frame_clahe = detector.detect(frame_original)
        
        # Encontra a base
        base_data = base_detector.detect(todos_shapes_detectados, frame_original)

        # FILTRAGEM DO ALVO
        alvos_encontrados = []
        for shape in todos_shapes_detectados:
            if shape['label'] == ALVO_SHAPE:
                alvos_encontrados.append(shape)
        
        # RASTREAMENTO (KALMAN) 
        if len(alvos_encontrados) > 0:
            # Pega o primeiro alvo encontrado para rastrear
            alvo_principal = alvos_encontrados[0] 
            print(f"Alvo '{ALVO_SHAPE}' localizado em {alvo_principal['center']}")
            
            # Alvo encontrado, corrige o filtro
            center_np = np.array([alvo_principal['center'][0], alvo_principal['center'][1]], np.float32)
            kalman_filter.correct(center_np)
            kalman_ativo = True
        
        if kalman_ativo:
            # Sempre prevê o próximo passo
            prediction = kalman_filter.predict()
            predicted_center = (int(prediction[0][0]), int(prediction[1][0]))


        #  VISUALIZAÇÃO 
        frame_com_desenho = frame_original.copy() 

        # Desenha TODAS as formas detectadas
        frame_com_desenho = detector.draw(frame_com_desenho, todos_shapes_detectados)

        # Desenho a base 
        if base_data is not None:
            center = base_data['center']
            radius = base_data['radius']
            cv2.circle(frame_com_desenho, center, radius, (255, 0, 0), 3) # Círculo azul
            cv2.circle(frame_com_desenho, center, 5, (0, 0, 255), -1)     # Centro real (vermelho)
            cv2.putText(frame_com_desenho, "BASE", (center[0] - 30, center[1] - radius - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        ### Desenho do Kalman 
        #if predicted_center:
            # Centro predito (verde)
            #cv2.circle(frame_com_desenho, predicted_center, 10, (0, 255, 0), 2)
            #cv2.putText(frame_com_desenho, f"{ALVO_SHAPE} Predito (Kalman)", (predicted_center[0] + 15, predicted_center[1]), 
                       # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow("RTSP Stream", frame_original)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.stop()
    cv2.destroyAllWindows()
    print("Detecção encerrada.")

if __name__ == "__main__":
    main()