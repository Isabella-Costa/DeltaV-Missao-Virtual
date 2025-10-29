import time
import math
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

CONNECTION_STRING = "tcp:127.0.0.1:5760"

def armar_drone_simplificado(drone, evento_abortar_seguranca=None):
    if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
        print("[ARM] Abortagem de segurança ativa. Cancelando armamento.")
        return False
    
    print("[ARM] Verificando se o drone está armável...")
    while not drone.is_armable:
        if evento_abortar_seguranca and evento_abortar_seguranca.is_set(): return False
        print("  Aguardando drone ficar armável...")
        time.sleep(1)
    print("[ARM] Drone está armável.")

    print("[ARM] Definindo modo GUIDED e armando...")
    drone.mode = VehicleMode("GUIDED")

    while drone.mode.name != 'GUIDED':
        if evento_abortar_seguranca and evento_abortar_seguranca.is_set(): return False
        time.sleep(0.5)

    drone.armed = True
    inicio_tempo = time.time()

    while not drone.armed and (time.time() - inicio_tempo < 10):
        if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
            drone.armed = False 
            print("[ARM] Abortagem durante armamento. Drone desarmado.")
            return False
        print("  Aguardando motores armarem...")
        time.sleep(0.5)
    
    if not drone.armed:
        print("[ARM] ERRO: Falha ou timeout ao armar.")
        return False
    
    print("[ARM] Motores ARMADOS com sucesso!")
    return True

def decolar_drone_simplificado(drone, altitude_alvo, evento_abortar_seguranca=None):
    if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
        print("[DECOLAR] Abortagem de segurança ativa. Decolagem cancelada.")
        return False
    
    if not drone.armed:
        print("[DECOLAR] ERRO: Drone não está armado. Arme-o antes de decolar.")
        return False

    print(f"\n[DECOLAR] Iniciando decolagem para {altitude_alvo:.1f} metros...")
    
    try:
        drone.simple_takeoff(altitude_alvo)
    except Exception as e:
        print(f"[DECOLAR] ERRO ao enviar comando de decolagem: {e}")
        return False

    inicio_tempo = time.time()
    max_tempo_espera = 60 
    while True:
        if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
            print("[DECOLAR] Abortagem de segurança ativa durante decolagem.")
            return False
        
        altitude_atual = drone.location.global_relative_frame.alt
        print(f"  [DECOLAR] Altitude: {altitude_atual:.1f} / {altitude_alvo:.1f}m")
        
        if altitude_atual >= altitude_alvo * 0.95:
            print(f"[DECOLAR] Altitude alvo ({altitude_alvo:.1f}m) alcançada.")
            break
        
        if (time.time() - inicio_tempo) > max_tempo_espera:
            print(f"[DECOLAR] ERRO: Timeout ({max_tempo_espera}s) na decolagem.")
            return False
        
        time.sleep(1)
    
    print("[DECOLAR] Decolagem concluída com sucesso!")
    return True

def liberar_carga_servo(drone, canal_servo, valor_pwm_liberar, valor_pwm_segurar, duracao_liberacao=2, evento_abortar_seguranca=None):
    if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
        print("[CARGA] Abortagem de segurança ativa. Liberação cancelada.")
        return False

    if not drone.armed:
        print("[CARGA] ERRO: Drone não está armado. Não é seguro liberar carga.")
        return False
    if drone.mode.name != "GUIDED":
        print("[CARGA] AVISO: Drone não está no modo GUIDED. Continue com cautela.")

    print(f"\n[CARGA] Liberando carga via servo no canal {canal_servo} (PWM: {valor_pwm_liberar})...")

    try:
        mensagem_liberar = drone.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, canal_servo, valor_pwm_liberar, 0, 0, 0, 0, 0)
        drone.send_mavlink(mensagem_liberar)
        print(f"[CARGA] Comando de liberação enviado. Servo em PWM: {valor_pwm_liberar}. Aguardando {duracao_liberacao}s...")

        inicio_tempo = time.time()
        while (time.time() - inicio_tempo) < duracao_liberacao:
            if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
                print("[CARGA] Abortagem de segurança durante liberação.")
               
                mensagem_segurar = drone.message_factory.command_long_encode(
                    0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, canal_servo, valor_pwm_segurar, 0, 0, 0, 0, 0)
                drone.send_mavlink(mensagem_segurar)
                return False
            time.sleep(0.1) 

        mensagem_segurar = drone.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, canal_servo, valor_pwm_segurar, 0, 0, 0, 0, 0)
        drone.send_mavlink(mensagem_segurar)
        time.sleep(0.5) 

        print("[CARGA] Carga liberada com sucesso e servo retornado para segurar.")
        return True

    except Exception as e:
        print(f"[CARGA] ERRO ao liberar carga via servo: {e}")
        return False
    import time

def pousar_drone_simplificado(drone, evento_abortar_seguranca=None):
    if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
        print("[POUSO] Abortagem de segurança ativa. Pouso cancelado.")
        return False

    if not drone.armed:
        print("[POUSO] ERRO: Drone não está armado. Não é possível pousar.")
        return False
    
    print("\n[POUSO] Iniciando procedimento de pouso (modo LAND)...")
    
    try:
        drone.mode = VehicleMode("LAND")
    except Exception as e:
        print(f"[POUSO] ERRO ao definir modo LAND: {e}")
        return False

    inicio_tempo = time.time()
    max_tempo_espera = 180 
    
    while drone.armed: 
        if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
            print("[POUSO] Abortagem de segurança ativa durante o pouso.")
            return False

        altitude_atual = drone.location.global_relative_frame.alt if drone.location.global_relative_frame else -1
        velocidade_vertical = drone.velocity[2] if drone.velocity else 0
        
        if altitude_atual > 0.1:
            print(f"  [POUSO] Altitude: {altitude_atual:.1f}m, Velocidade vertical: {velocidade_vertical:.1f}m/s")
        else:
            print("  [POUSO] Próximo ao solo...")

        if (time.time() - inicio_tempo) > max_tempo_espera:
            print(f"[POUSO] ERRO: Timeout ({max_tempo_espera}s) no pouso.")
            if drone.armed: 
                print("[POUSO] Tentando desarmar drone após timeout...")
                drone.armed = False
                time.sleep(2)
            return False

        time.sleep(1) 

    print("[POUSO] Drone pousou e desarmou com sucesso!")
    return True

CONNECTION_STRING = "udp:127.0.0.1:14550"
safety_abort = False
vehicle = None 
CRUISE_GROUNDSPEED = 2

def obter_localização_atual_relativa():
    """Retorna a localização atual do drone em relação ao ponto de decolagem."""
    if vehicle and vehicle.location.local_frame:
        print("Aviso: get_current_location_relative usando LocationGlobalRelative. Implementação real necessitaria de um ponto 'home' fixo ou sistema de coordenadas local.")
        return vehicle.location.global_frame

def velocidade_enviada_ned(velocity_x, velocity_y, velocity_z, duration):

    if safety_abort: return
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, 
        0, 0)    

    for x in range(0, int(duration)):
        if safety_abort: return
        vehicle.send_mavlink(msg)
        time.sleep(1)

def ir_para_local(localização_alvo, groundspeed=CRUISE_GROUNDSPEED):
    if safety_abort: return False
    print(f"Voando para: {localização_alvo} a {groundspeed} m/s")
    vehicle.groundspeed = groundspeed
    vehicle.simple_goto(localização_alvo)

    while True:
        if safety_abort: return False
        current_location = vehicle.location.global_relative_frame
        distancia = get_distance_metres(current_location, localização_alvo)
        print(f" Distância até o alvo: {distancia:.2f} metros")
        if distancia < 1: 
            print("Alvo alcançado.")
            break
        time.sleep(1)
    return True

def get_distance_metres(alocalizção1, alocalização2):
    distancialat = alocalização2.lat - alocalizção1.lat
    distancialong = alocalização2.lon - alocalizção1.lon
    return math.sqrt((distancialat**2) + (distancialong**2)) * 1.113195e5