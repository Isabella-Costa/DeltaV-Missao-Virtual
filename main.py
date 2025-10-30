import cv2
import numpy as np
from dronekit import connect
import time # Adicionado para o sleep

from controle import armar_drone_simplificado, decolar_drone_simplificado
from controle import pousar_drone_simplificado

from detectors.shapeDetection import ShapeDetector, incializar_kalman
from detectors.baseDetection import BaseDetector

from camera_sim import Camera

drone = connect("udp:127.0.0.1:14550", wait_ready = True)


def extrairCaracteristica(cnt, focal_length_pixels, known_width_cm):

    caracteristicas = {}

    # Largura em pixels (usando Bounding Box simples)
    x, y, w, h = cv2.boundingRect(cnt)
    pixel_width = w
    caracteristicas['pixel_width'] = pixel_width

    # Distancia_cm = (Largura_Real_cm * Distancia_Focal_Pixels) / Largura_Em_Pixels
    distancia_cm = 0.0
    if pixel_width > 0:
        distancia_cm = (known_width_cm * focal_length_pixels) / pixel_width
    
    caracteristicas['distancia_cm'] = distancia_cm
    
    return caracteristicas

def processar_dados_alvo(shape_detectada, focal_length, known_width):
    
    # Extraindo informações necessarias
    centro = shape_detectada.get('center')
    area = shape_detectada.get('area')
    contorno = shape_detectada.get('contour')
    label = shape_detectada.get('label')

    if contorno is None:
        return None # Não processa sem contorno

    caracteristicas_controle = extrairCaracteristica(contorno, focal_length, known_width)

    # dicionário do alvo
    dados_do_alvo = {
        'label': label,
        'centro': centro,
        'area': area,
        'distancia_cm': caracteristicas_controle.get('distancia_cm', 0.0),
        'pixel_width': caracteristicas_controle.get('pixel_width', 0),
        'bounding_box': caracteristicas_controle.get('bounding_box', (0,0,0,0))
    }
    
    dados_completos = dados_do_alvo.copy()
    dados_completos['contour'] = contorno
    
    return dados_completos


def main():

    # CONFIGURAÇÃO DO ALVO 
    ALVO_SHAPE = "Estrela" 

    try:
        cam = Camera()
    except IOError as e:
        print(e)
        return

    # Parâmetros da Câmera 
    H_PIXELS = cam.H_PIXELS  # 640
    H_FOV_DEG = cam.H_FOV_DEG # 62.2
    
    # Calcular distância focal em pixels
    fov_rad = H_FOV_DEG * (np.pi / 180)
    FOCAL_LENGTH_PIXELS = (H_PIXELS / 2) / np.tan(fov_rad / 2)
    print(f"Distância Focal da Câmera (calculada): {FOCAL_LENGTH_PIXELS:.2f} pixels")

    #LARGURA DA IMG
    WIDTH_CM = 80.0  # (cm) -



    #  Inicializa os detectores e o Kalman 
    detector = ShapeDetector()       
    base_detector = BaseDetector() 
    kalman_filter = incializar_kalman()
    kalman_ativo = False
    predicted_center = None 
    
    cam.start() 
    
    # --- Comandos do Drone ---
    armar_drone_simplificado(drone)
    decolar_drone_simplificado(drone, 4)
    print("Iniciando modo de detecção (Drone em espera).")
    print(f"Iniciando modo de detecção. Alvo: {ALVO_SHAPE}. Pressione 'q' para sair.")

    while True:
        # Frame original
        ret, frame_original = cam.read() 
        if not ret:
            print("Frame não capturado ou vídeo terminou.")
            break
        
        # DETECÇÃO
        todos_shapes_detectados, bordas_canny, frame_clahe = detector.detect(frame_original)
        
        # Encontra a base
        base_data = base_detector.detect(todos_shapes_detectados, frame_original)

        # FILTRAGEM DO ALVO E EXTRAÇÃO DE CARACTERÍSTICAS
        alvos_encontrados = []
        for shape in todos_shapes_detectados:
            if shape['label'] == ALVO_SHAPE:
                
                # Pegar o contorno do alvo
                cnt = shape['contour']

                # Extrair características de controle (distância, etc.)
                caracteristicas_controle = extrairCaracteristica(cnt, FOCAL_LENGTH_PIXELS, KNOWN_WIDTH_CM)
                
                # Adicionar essas características ao dicionário do alvo
                shape.update(caracteristicas_controle)
                
                # Adicionar à lista de alvos
                alvos_encontrados.append(shape)
        
        
        # RASTREAMENTO (KALMAN) 
        if len(alvos_encontrados) > 0:
            # Pega o primeiro alvo encontrado para rastrear
            alvo_principal = alvos_encontrados[0] 
            
            # --- DADOS PARA O CONTROLE ---
            centro_alvo = alvo_principal['center']
            distancia_alvo = alvo_principal['distancia_cm']
            
            print(f"ALVO '{ALVO_SHAPE}' DETECTADO: Centro {centro_alvo} | Distância: {distancia_alvo:.1f} cm")

            # =========================================
            #
            # (Aqui entraria sua lógica de controle do drone 
            #
            # =========================================

            
            # Alvo encontrado, corrige o filtro
            center_np = np.array([centro_alvo[0], centro_alvo[1]], np.float32)
            kalman_filter.correct(center_np)
            kalman_ativo = True
        
        else:
            # Informa que o alvo não foi encontrado 
            print(f"Procurando alvo '{ALVO_SHAPE}'...")


        if kalman_ativo:
            # Sempre prevê o próximo passo
            prediction = kalman_filter.predict()
            predicted_center = (int(prediction[0][0]), int(prediction[1][0]))


        #  VISUALIZAÇÃO 
        frame_com_desenho = frame_original.copy() 

        # Desenha TODAS as formas detectadas 
        frame_com_desenho = detector.draw(frame_com_desenho, todos_shapes_detectados)


        # Desenha a distância apenas nos alvos que filtramos
        for alvo in alvos_encontrados:
            x, y, _, _ = cv2.boundingRect(alvo['contour'])
            dist_text = f"Dist: {alvo['distancia_cm']:.1f} cm"
            
            cv2.putText(frame_com_desenho, dist_text, (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Desenho a base 
        if base_data is not None:
            center = base_data['center']
            radius = base_data['radius']
            cv2.circle(frame_com_desenho, center, radius, (255, 0, 0), 3) # Círculo azul
            cv2.circle(frame_com_desenho, center, 5, (0, 0, 255), -1)     # Centro real (vermelho)
            cv2.putText(frame_com_desenho, "BASE", (center[0] - 30, center[1] - radius - 10),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        
        cv2.imshow("RTSP Stream", frame_com_desenho) # Mostra o frame com desenhos
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #pousar_drone_simplificado(drone)
    cam.stop()
    cv2.destroyAllWindows()
    print("Detecção encerrada.")

if __name__ == "__main__":
    main()
