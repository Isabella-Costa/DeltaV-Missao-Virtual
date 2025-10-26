import cv2
import numpy as np

from detectors.cameraCapture import CameraCapture
from detectors.shapeDetection import ShapeDetector
from detectors.baseDetection import BaseDetector
from utils.tracker import Tracker
from utils.visualizer import Visualizer

def find_target(all_shapes, target_label):
    for shape in all_shapes:
        if shape['label'] == target_label:
            return shape
    return None


def main():
    # CONFIGURAÇÃO DO ALVO 
    ALVO_SHAPE = "Hexágono" 

    try:
        camera = CameraCapture(source=0) 
    except IOError as e:
        print(e)
        return

    #  Inicializa os detectores e o Kalman 
    detector = ShapeDetector()      
    base_detector = BaseDetector() 
    tracker = Tracker()
    visualizer = Visualizer(target_label=ALVO_SHAPE)

    print(f"Iniciando detecção. Alvo: {ALVO_SHAPE}. Pressione 'q para sair")

    while True:
        ret, frame_original = camera.get_frame()
        if not ret:
            print("Frame não capturado ou video terminou")
            break
        
        todos_shapes, bordas_canny, frame_clahe = detector.detect(frame_original)

        base_data = base_detector.detect(todos_shapes, frame_original)

        alvos_encontrado = find_target(todos_shapes, ALVO_SHAPE)

        centro_alvo = alvos_encontrado['center'] if alvos_encontrado else None
        previsao = tracker.update(target_found=(alvos_encontrado is not None), center=centro_alvo)

        visualizer.display_frame(
            frame_original,
            bordas_canny,
            frame_clahe,
            todos_shapes,
            base_data,
            previsao,
            ALVO_SHAPE
        )

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    camera.release()
    cv2.destroyAllWindows()
    print("Detecção encerrada")

if __name__ == "__main__":
        main()
'''
    # Configuração janela
    window_name_normal = f"{JANELA_CONFIG['WINDOW_NAME_NORMAL_PREFIX']} [{ALVO_SHAPE}]"
    window_name_canny = JANELA_CONFIG["WINDOW_NAME_CANNY"]
    window_name_clahe = JANELA_CONFIG["WINDOW_NAME_CLAHE"]
    
    cv2.namedWindow(window_name_normal, cv2.WINDOW_NORMAL)
    cv2.namedWindow(window_name_canny, cv2.WINDOW_NORMAL)
    cv2.namedWindow(window_name_clahe, cv2.WINDOW_NORMAL)

    largura = JANELA_CONFIG["LARGURA_RESIZE"] 
    altura = int(largura * (camera.height / camera.width))
    
    dim = (largura, altura) 
    
    cv2.resizeWindow(window_name_normal, largura, altura)
    cv2.resizeWindow(window_name_canny, largura, altura)
    cv2.resizeWindow(window_name_clahe, largura, altura)

    print(f"Iniciando modo de detecção. Alvo: {ALVO_SHAPE}. Pressione 'q' para sair.")


    print(f"Iniciando modo de detecção. Alvo: {ALVO_SHAPE}. Pressione 'q' para sair.")

    while True:
        # Frame original
        ret, frame_original = camera.get_frame() 
        if not ret:
            print("Frame não capturado ou vídeo terminou.")
            break
        
        # REDIMENSIONA O FRAME 
        frame = cv2.resize(frame_original, dim, interpolation=cv2.INTER_AREA)
        todos_shapes_detectados, bordas_canny, frame_clahe = detector.detect(frame)
        
        # Encontra a base
        base_data = base_detector.detect(todos_shapes_detectados, frame)

        # FILTRAGEM 
        alvos_encontrados = []
        for shape in todos_shapes_detectados:
            if shape['label'] == ALVO_SHAPE:
                alvos_encontrados.append(shape)
        
        if alvos_encontrados:
             print(f"Alvo '{ALVO_SHAPE}' localizado em {alvos_encontrados[0]['center']}")


        # RASTREAMENTO (KALMAN) 
        predicted_center = None
        if len(alvos_encontrados) > 0:
            alvo_principal = alvos_encontrados[0] 
            center_np = np.array([alvo_principal['center'][0], alvo_principal['center'][1]], np.float32)
            kalman_filter.correct(center_np)
            kalman_ativo = True
        
        if kalman_ativo:
            prediction = kalman_filter.predict()
            predicted_center = (int(prediction[0][0]), int(prediction[1][0]))


        #  VISUALIZAÇÃO 
        frame_com_desenho = frame.copy() 

        # Desenha TODAS as formas detectadas
        frame_com_desenho = detector.draw(frame_com_desenho, todos_shapes_detectados)

        # FILTRAGEM 
        alvos_encontrados = []
        for shape in todos_shapes_detectados:
            if shape['label'] == ALVO_SHAPE:
                alvos_encontrados.append(shape)
                print(f"Alvo '{ALVO_SHAPE}' localizado em {shape['center']}")

        predicted_center = None

        if len(alvos_encontrados) > 0:
            # Pega o primeiro alvo encontrado para rastrear
            alvo_principal = alvos_encontrados[0] 
            
            # Alvo encontrado, corrige o filtro
            center_np = np.array([alvo_principal['center'][0], alvo_principal['center'][1]], np.float32)
            kalman_filter.correct(center_np)
            kalman_ativo = True
        
        if kalman_ativo:
            # Sempre prevê o próximo passo
            prediction = kalman_filter.predict()
            predicted_center = (int(prediction[0][0]), int(prediction[1][0]))


        # VISUALIZAÇÃO 
        frame_com_desenho = frame.copy()

        # Desenha TODAS as formas detectadas 
        frame_com_desenho = detector.draw(frame_com_desenho, todos_shapes_detectados)

        # Desenho a base 
        if base_data is not None:
            center = base_data['center']
            radius = base_data['radius']
            cv2.circle(frame_com_desenho, center, radius, (255, 0, 0), 3) # Círculo azul
            cv2.circle(frame_com_desenho, center, 5, (0, 0, 255), -1)    # Centro real (vermelho)
            cv2.putText(frame_com_desenho, "BASE", (center[0] - 30, center[1] - radius - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Kalman 
        if predicted_center:
            # Centro predito (verde)
            cv2.circle(frame_com_desenho, predicted_center, 10, (0, 255, 0), 2)
            cv2.putText(frame_com_desenho, f"{ALVO_SHAPE} Predito (Kalman)", (predicted_center[0] + 15, predicted_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Mostra os resultados
        cv2.imshow(f"Visao do Drone - Procurando por [{ALVO_SHAPE}]", frame_com_desenho)
        cv2.imshow("Debug Canny", bordas_canny)
        cv2.imshow("Debug CLAHE", frame_clahe)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()
    print("Detecção encerrada.")

if __name__ == "__main__":
    main()
'''