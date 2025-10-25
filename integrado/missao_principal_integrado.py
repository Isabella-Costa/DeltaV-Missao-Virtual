import cv2 as cv
import numpy as np
import time
import threading

from nova_visao import detect_target
from controle_simplificado import (connect, armar_drone_simplificado, 
                            decolar_drone_simplificado, velocidade_enviada_ned, 
                            pousar_drone_simplificado, safety_abort, vehicle, 
                            CONNECTION_STRING, threading, mavutil)

TARGET_ALTITUDE = 5  # Altitude alvo para decolagem
CRUISE_SPEED = 0.5   # Velocidade de aproximação (m/s)
KP = 0.005           # Ganho Proporcional para o controle P
CENTERING_TOLERANCE_PX = 30 # Tolerância para o erro em pixels

def send_velocity_command(vx, vy, vz, duration=0.1):
    # Adapta a função do seu arquivo de controle
    # MAV_FRAME_LOCAL_NED = sistema de coordenadas local
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
    )
    # Envia o comando rapidamente para atualizações em tempo real
    vehicle.send_mavlink(msg)
    time.sleep(duration) # Aguarda um breve período antes da próxima leitura

def run_mission():
    global vehicle
    
    # 1. CONEXÃO E CÂMERA
    vehicle = connect(CONNECTION_STRING, wait_ready=True)
    abort_event = threading.Event() 
    cap = cv.VideoCapture(1) # Abre a câmera (ou stream do Webots/SITL)
    
    if not cap.isOpened():
        print("Erro: Não foi possível acessar a câmera.")
        return

    # 2. FASE DE VOO INICIAL
    # if not armar_drone_simplificado(vehicle, abort_event): return
    # if not decolar_drone_simplificado(vehicle, TARGET_ALTITUDE, abort_event): return
    
    # 3. LOOP DE VISÃO E CONTROLE
    print("\n--- INICIANDO LOOP DE VISÃO E CONTROLE ---")
    
    try:
        while not abort_event.is_set():
            ret, frame = cap.read()
            if not ret: continue
            
            height, width, _ = frame.shape
            center_x_ref = int(width / 2) # Centro de referência X
            center_y_ref = int(height / 2) # Centro de referência Y
            
            # --- CHAMA A FUNÇÃO DE VISÃO ---
            detection = detect_target(frame, center_x_ref, center_y_ref)
            
            # --- LÓGICA DE CONTROLE BASEADA NA DETECÇÃO ---
            if detection['found']:
                error_x = detection['error_x']
                error_y = detection['error_y']
                
                # Controle Proporcional (P)
                vx = -error_y * KP # Erro vertical (Y) afeta o movimento p/ frente/trás (X)
                vy = -error_x * KP # Erro horizontal (X) afeta o movimento lateral (Y)
                
                # Limitação de velocidade
                max_vel = CRUISE_SPEED
                vx = np.clip(vx, -max_vel, max_vel)
                vy = np.clip(vy, -max_vel, max_vel)

                # Envia comando de velocidade (mantém altitude (vz=0))
                send_velocity_command(vx, vy, 0)
                
                # LÓGICA DE TRANSIÇÃO (Centralização atingida)
                if abs(error_x) < CENTERING_TOLERANCE_PX and abs(error_y) < CENTERING_TOLERANCE_PX:
                    print("ALVO CENTRALIZADO. PRONTO PARA PRÓXIMA ETAPA (POUSO/DESCIDA).")
                    # Lógica para pousar ou avançar aqui
            else:
                # Se não encontrar nada, o drone pode parar ou procurar (ex: velocidade_enviada_ned(0, 0, 0, 0.1))
                send_velocity_command(0, 0, 0) 
            
            # --- Opcional: Desenho e Interface (para DEBUG) ---
            # Se você ainda quiser ver o frame processado para debug, pode desenhar 
            # *APENAS NESTE LOOP* e não na função detect_target.
            cv.putText(frame, f"Status: {detection['figure_name']}", (20, 50), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv.imshow('Missão Autonoma', frame)

            if cv.waitKey(1) & 0xFF == ord('q'):
                abort_event.set()
                break

    except KeyboardInterrupt:
        abort_event.set()
    
    finally:
        # 4. ENCERRAMENTO
        print("\n--- ENCERRANDO MISSÃO ---")
        # pousar_drone_simplificado(vehicle, abort_event)
        vehicle.close()
        cap.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    run_mission()


