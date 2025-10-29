import time
import math
import threading
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# --- Configurações Globais---
STRING_CONEXAO = "udp:127.0.0.1:14550" 

# --- Funções Auxiliares ---

def distancia_metros(local_1, local_2):
    delta_latitude = local_2.lat - local_1.lat
    delta_longitude = local_2.lon - local_1.lon
    return math.sqrt((delta_latitude*delta_latitude) + (delta_longitude*delta_longitude)) * 1.113195e5

def velocidade(velocidade_x, velocidade_y, velocidade_z, duracao_s):
    mensagem = veiculo.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocidade_x, velocidade_y, velocidade_z, # x, y, z velocity (m/s)
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)

    for i in range(0, int(duracao_s)):
        if evento_abortar_seguranca.is_set():
            print("[VELOCIDADE_NED] Abortagem de segurança durante movimento. Interrompendo.")
            return
        veiculo.send_mavlink(mensagem)
        time.sleep(1)


# --- Funções de Controle do Drone (Simplificadas) ---

def armar(drone):
    
    print("\n[ARM] Verificando se o drone está armável...")
    while not drone.is_armable:
        print("  Aguardando drone ficar armável...")
        time.sleep(1)
    print("[ARM] Definindo modo GUIDED e armando...")
    drone.mode = VehicleMode("GUIDED")

    while drone.mode.name != 'GUIDED':
        print("  Aguardando modo GUIDED...")
        time.sleep(0.5)
    drone.armed = True
    tempo_inicio = time.time()

    while not drone.armed and (time.time() - tempo_inicio < 10):
        print("  Aguardando motores armarem...")
        time.sleep(0.5)
     
    if not drone.armed:
        print("[ARM] ERRO: Falha ou timeout ao armar.")
        return False
    
    print("[ARM] Motores ARMADOS com sucesso!")
    return True

def decolar(drone, altitude_alvo):
    if not drone.armed:
        print("[DECOLAR] ERRO: Drone não está armado. Arme-o antes de decolar.")
        return False

    print(f"\n[DECOLAR] Iniciando decolagem para {altitude_alvo:.1f} metros...")
    
    try:
        drone.simple_takeoff(altitude_alvo)
    except Exception as e:
        print(f"[DECOLAR] ERRO ao enviar comando de decolagem: {e}")
        return False

    tempo_inicio = time.time()
    tempo_max_espera = 60 # segundos  
        altitude_atual = drone.location.global_relative_frame.alt
        print(f"  [DECOLAR] Altitude: {altitude_atual:.1f} / {altitude_alvo:.1f}m")
        
        if altitude_atual >= altitude_alvo * 0.95: # 95% da altitude alvo
            print(f"[DECOLAR] Altitude alvo ({altitude_alvo:.1f}m) alcançada.")
            break
        
        if (time.time() - tempo_inicio) > tempo_max_espera:
            print(f"[DECOLAR] ERRO: Timeout ({tempo_max_espera}s) na decolagem.")
            return False
        
        time.sleep(1)
    
    print("[DECOLAR] Decolagem concluída com sucesso!")
    return True

def pousar(drone):
    if not drone.armed:
        print("[POUSO] ERRO: Drone não está armado. Não é possível pousar.")
        return False
    
    print("\n[POUSO] Iniciando procedimento de pouso (modo LAND)...")
    
    try:
        drone.mode = VehicleMode("LAND")
    except Exception as e:
        print(f"[POUSO] ERRO ao definir modo LAND: {e}")
        return False

    tempo_inicio = time.time()
    tempo_max_espera = 180 # segundos
    
    while drone.armed: # O drone desarma automaticamente ao pousar em modo LAND
        if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
            print("[POUSO] Abortagem de segurança ativa durante o pouso.")
            # Para abortar um pouso, o ideal seria mudar para GUIDED e subir, mas para simplificar, apenas retornamos.
            # Em um cenário real, você adicionaria lógica para transicionar para um estado seguro.
            return False

        altitude_atual = drone.location.global_relative_frame.alt if drone.location.global_relative_frame else -1
        # veiculo.velocity retorna [vx, vy, vz] onde vz é velocidade DOWN, então positivo é para baixo.
        velocidade_vertical = drone.velocity[2] if drone.velocity else 0
        
        if altitude_atual > 0.3: # Acima de 30cm do chão
            print(f"  [POUSO] Altitude: {altitude_atual:.1f}m, Velocidade vertical: {velocidade_vertical:.1f}m/s")
        else:
            print("  [POUSO] Próximo ao solo...")

        if (time.time() - tempo_inicio) > tempo_max_espera:
            print(f"[POUSO] ERRO: Timeout ({tempo_max_espera}s) no pouso.")
            if drone.armed: 
                print("[POUSO] Tentando desarmar drone após timeout...")
                drone.armed = False
                time.sleep(2)
            return False

        time.sleep(1) 

    print("[POUSO] Drone pousou e desarmou com sucesso!")
    return True

#--- Monitor de Abortagem de Segurança ---
def abortagem(evento):
    print("\n[SEGURANCA] Monitoramento de abortagem ativado. Pressione 'q' + Enter a qualquer momento para abortar.")
    while not evento.is_set():
        try:
            caractere_entrada = input()
            if caractere_entrada.lower() == 'q':
                print("\n[SEGURANCA] Comando de abortagem 'q' recebido! Ativando evento de segurança.")
                evento.set()
                return True
                break
            else:
                print("[SEGURANCA] Comando inválido. Pressione 'q' para abortar.")
        except EOFError:
            print("[SEGURANCA] Entrada padrão fechada. Monitor de abortagem encerrado.")
            break
        except Exception as e:
            print(f"[SEGURANCA] Erro no monitor de abortagem: {e}")
            break

