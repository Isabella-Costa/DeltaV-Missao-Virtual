import cv2
import numpy as np

from detectors.shapeDetection import ShapeDetector, incializar_kalman
from detectors.baseDetection import BaseDetector
from src.capture.camera_sim import Camera

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from src.controle import velocidade, armar, decolar, distancia_metros, pousar
 # --- Configurações Globais---
STRING_CONEXAO = "udp:127.0.0.1:14550" 
drone=STRING_CONEXAO
                


def extrairCaracteristica(cnt, focal_length_pixels, known_width_cm):

    caracteristicas = {}

    # Largura em pixels
    x, y, w, h = cv2.boundingRect(cnt)
    pixel_width = w
    caracteristicas['pixel_width'] = pixel_width
    caracteristicas['bounding_box'] = (x, y, w, h)

    # Calcular Distância
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

    drone = connect("udp:127.0.0.1:14550", wait_ready = True)

    # CONFIGURAÇÃO DO ALVO 
    ALVO_SHAPE = "Estrela" 

    try:
        cam = Camera()
    except IOError as e:
        print(f"Erro ao iniciar a câmera: {e}")
        return

    # PARÂMETROS DA CÂMERA E DO ALVO
    H_PIXELS = cam.H_PIXELS  # 640
    H_FOV_DEG = cam.H_FOV_DEG # 62.2
    
    fov_rad = H_FOV_DEG * (np.pi / 180)
    FOCAL_LENGTH_PIXELS = (H_PIXELS / 2) / np.tan(fov_rad / 2)
    print(f"Distância Focal da Câmera (calculada): {FOCAL_LENGTH_PIXELS:.2f} pixels")


    KNOWN_WIDTH_CM = 10.0  # (cm) - Largura real do seu alvo
    # -------------------------------------


    #  Detectores e o Kalman 
    detector = ShapeDetector()       
    base_detector = BaseDetector() 
    kalman_filter = incializar_kalman()
    kalman_ativo = False
    predicted_center = None 
    
    cam.start() 
    
    # --- Comandos do Drone ---
      # ----Armando----
    if estado == "armando":
        print("Drone sendo armado")
        armar(drone)
        if drone.armed == True:
          estado="decolando"
        # ----Decolando----
    elif estado=="decolando":
        print("Drone Decolando para uma altitude de 5 metros")
        decolar(drone,5)
        if decolar(drone,5)==True:
          estado="vasculhar"
    print("Iniciando modo de detecção (Drone em espera).")
    print(f"Iniciando modo de detecção. Alvo: {ALVO_SHAPE}. Pressione Ctrl+C para sair.")
    try:
        while True:
            # Frame original
            ret, frame_original = cam.read() 
            if not ret:
                print("Frame não capturado ou vídeo terminou.")
                break
            
            # DETECÇÃO
            todos_shapes_detectados, bordas_canny, frame_clahe = detector.detect(frame_original)
            
            base_data = base_detector.detect(todos_shapes_detectados, frame_original)

            # FILTRAGEM E PROCESSAMENTO DO ALVO
            alvos_encontrados = []
            for shape in todos_shapes_detectados:
                if shape['label'] == ALVO_SHAPE:
                    
                    dados_alvo = processar_dados_alvo(shape, FOCAL_LENGTH_PIXELS, KNOWN_WIDTH_CM)
                    
                    if dados_alvo:
                        alvos_encontrados.append(dados_alvo)
            
            
            # RASTREAMENTO (KALMAN) 
            if len(alvos_encontrados) > 0:
                alvo_principal = alvos_encontrados[0] 
                centro_alvo = alvo_principal['centro']
                estado="centralizando"
                # --- DEBUG
                dados_para_print = alvo_principal.copy()
                dados_para_print.pop('contour', None) 
                
                print(f"DADOS DO ALVO: {dados_para_print}")
                    # ----Centralizando----
                if estado=="centralizando":
                      while True:
                        if abs(distancia_metros(drone.location.global_frame, centro_alvo))>0.2:
                           velocidade(0.5,0.5,0,1)
                        else:
                          estado="pousando"
                          break
                    # ----pousando-----
                elif estado=="pousando":
                    print("Drone pousando")
                    pousar(drone)
                    if pousar(drone)==True:
                        estado="decolar1"
                # ----decolando1----
                elif estado=="decolar1":
                    print("Decolando o drone")
                    decolar(drone,5)
                    if decolar(drone,5)==True:
                        estado="rtl"
                # ----voltando para casa------
                elif estado=="rtl":
                    print("voltando par casa")
                    while not drone.mode.name == "RTL":
                        print(" Aguardando a mudança de modo...")
                    time.sleep(1)
                    print("Modo alterado para RTL com sucesso!")
                    print("O veículo agora está retornando e pousando...")
                    print("Aguardando o pouso...")
                    drone.close()
                    print("Conexão fechada.")
                # Alvo encontrado, corrige o filtro
                center_np = np.array([centro_alvo[0], centro_alvo[1]], np.float32)
                kalman_filter.correct(center_np)
                kalman_ativo = True
            
            elif len(alvos_encontrados) == 0:
                                # ----vasculhando-----
                if estado=="vasculhar":
                 if len(alvos_encontrados)>0:
                     estado=="centralizando"
                 for i in range(9):
                   if i % 2 ==0 and len(alvos_encontrados) == 0:
                     velocidade(0,-1,0,9)
                     print("drone se deslocando para a esquerda em 1 metro por segundo por 9 segundos")
                     velocidade(1,0,0,1)
                     print("drone se deslocando para frente em 1 metro por segundo por 1 segundo")
                     time.sleep(3)
                   elif i % 2 !=0 and len(alvos_encontrados) == 0:
                      velocidade(0,1,0,9)
                      print("drone se deslocando para a direita em 1 metro por segundo por 9 segundos")
                      velocidade(1,0,0,1)
                      print("drone se deslocando para frente em 1 metro por segundo por 1 segundo")
                      time.sleep(3)
            
            if kalman_ativo:
                prediction = kalman_filter.predict()
                predicted_center = (int(prediction[0][0]), int(prediction[1][0]))


            time.sleep(0.01) 

    except KeyboardInterrupt:
        print("\nRecebido comando de parada (Ctrl+C). Encerrando...")
    
    finally:
        # pousar_drone_simplificado(drone)
        cam.stop()
        print("Detecção encerrada.")
        

if __name__ == "__main__":
    main()