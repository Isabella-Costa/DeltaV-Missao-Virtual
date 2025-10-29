import time
import math
import threading
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# --- Configurações Globais ---
STRING_CONEXAO = "udp:127.0.0.1:14550" # Ou 'tcp:127.0.0.1:5760' ou porta serial, dependendo da sua configuração
ALTITUDE_DECOLAGEM_PADRAO = 3  # Metros (você pode ajustar conforme a missão)
VELOCIDADE_CRUZEIRO_SOLO = 2       # m/s

# Evento para sinalizar abortagem de segurança
evento_abortar_seguranca = threading.Event()
# Objeto do veículo (drone)
veiculo = None

# --- Funções Auxiliares ---

def obter_distancia_metros(local_1, local_2):
    """
    Retorna a distância em metros entre duas localizações GPS.
    Esta é uma aproximação para pequenas distâncias.
    """
    delta_latitude = local_2.lat - local_1.lat
    delta_longitude = local_2.lon - local_1.lon
    return math.sqrt((delta_latitude*delta_latitude) + (delta_longitude*delta_longitude)) * 1.113195e5

def enviar_velocidade_ned(velocidade_x, velocidade_y, velocidade_z, duracao_s):
    """
    Envia comandos de velocidade no frame NED (North, East, Down) relativo ao corpo do drone.
    velocidade_x: Velocidade para frente (m/s)
    velocidade_y: Velocidade para a direita (m/s)
    velocidade_z: Velocidade para baixo (m/s)
    duracao_s: Duração do comando em segundos
    """
    if evento_abortar_seguranca.is_set():
        print("[VELOCIDADE_NED] Abortagem de segurança ativa. Comando de velocidade cancelado.")
        return

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

def enviar_alvo_posicao_ned(norte, leste, baixo, guinada=0.0):
    """
    Envia um comando para mover o drone para uma posição específica (North, East, Down)
    no frame LOCAL_NED (relativo ao ponto de HOME).
    norte: Posição no eixo norte (m)
    leste: Posição no eixo leste (m)
    baixo: Posição no eixo para baixo (m) (altitude_alvo * -1)
    guinada: Ângulo de guinada desejado (radianos)
    """
    if evento_abortar_seguranca.is_set():
        print("[ALVO_POS_NED] Abortagem de segurança ativa. Comando de posição cancelado.")
        return False
    
    # Para usar MAV_FRAME_LOCAL_NED, é necessário que o drone tenha um HOME position set.
    # Em simulações (SITL), o HOME é geralmente onde o simulador inicia.
    
    mensagem = veiculo.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b000111111000, # type_mask (only position enabled)
        norte, leste, baixo, # x, y, z positions (m)
        0, 0, 0, # x, y, z velocity (not used)
        0, 0, 0, # x, y, z acceleration (not used)
        guinada, 0)  # yaw, yaw_rate (yaw enabled, yaw_rate not used)

    veiculo.send_mavlink(mensagem)
    return True

def ir_para_localizacao_global_relativa(localizacao_alvo, velocidade_solo=VELOCIDADE_CRUZEIRO_SOLO):
    """
    Move o drone para uma localização GPS global relativa ao ponto de decolagem.
    localizacao_alvo: Objeto LocationGlobalRelative do destino.
    velocidade_solo: Velocidade de solo desejada (m/s).
    """
    if evento_abortar_seguranca.is_set():
        print("[IR_GLOBAL] Abortagem de segurança ativa. Comando de ir para localização cancelado.")
        return False
    
    print(f"\n[IR_GLOBAL] Voando para: {localizacao_alvo} a {velocidade_solo} m/s...")
    veiculo.groundspeed = velocidade_solo
    
    try:
        veiculo.simple_goto(localizacao_alvo)
    except Exception as e:
        print(f"[IR_GLOBAL] ERRO ao enviar comando simple_goto: {e}")
        return False

    tempo_inicial = time.time()
    while True:
        if evento_abortar_seguranca.is_set():
            print("[IR_GLOBAL] Abortagem de segurança durante navegação. Interrompendo.")
            veiculo.groundspeed = 0 # Parar o drone
            return False

        localizacao_atual = veiculo.location.global_relative_frame
        distancia = obter_distancia_metros(localizacao_atual, localizacao_alvo)
        
        # Logica para evitar loop infinito se o drone estiver muito longe
        if (time.time() - tempo_inicial) > 300: # 5 minutos de timeout
            print("[IR_GLOBAL] ERRO: Timeout na navegação para o alvo. Possível problema.")
            return False

        print(f"  [IR_GLOBAL] Distância até o alvo: {distancia:.2f} metros (lat: {localizacao_atual.lat:.6f}, lon: {localizacao_atual.lon:.6f}, alt: {localizacao_atual.alt:.2f}m)")
        if distancia < 1:  # Dentro de 1 metro do alvo
            print("[IR_GLOBAL] Alvo alcançado.")
            break
        time.sleep(1)
    return True

# --- Funções de Controle do Drone (Simplificadas) ---

def armar_drone_simplificado(drone, evento_abortar_seguranca=None):
    if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
        print("[ARM] Abortagem de segurança ativa. Cancelando armamento.")
        return False
    
    print("\n[ARM] Verificando se o drone está armável...")
    while not drone.is_armable and not (evento_abortar_seguranca and evento_abortar_seguranca.is_set()):
        print("  Aguardando drone ficar armável...")
        time.sleep(1)
    if evento_abortar_seguranca and evento_abortar_seguranca.is_set(): return False
    print("[ARM] Drone está armável.")

    print("[ARM] Definindo modo GUIDED e armando...")
    drone.mode = VehicleMode("GUIDED")

    while drone.mode.name != 'GUIDED' and not (evento_abortar_seguranca and evento_abortar_seguranca.is_set()):
        print("  Aguardando modo GUIDED...")
        time.sleep(0.5)
    if evento_abortar_seguranca and evento_abortar_seguranca.is_set(): return False

    drone.armed = True
    tempo_inicio = time.time()

    while not drone.armed and (time.time() - tempo_inicio < 10) and not (evento_abortar_seguranca and evento_abortar_seguranca.is_set()):
        print("  Aguardando motores armarem...")
        time.sleep(0.5)
    
    if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
        if drone.armed:
            drone.armed = False 
            print("[ARM] Abortagem durante armamento. Drone desarmado.")
        return False
    
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

    tempo_inicio = time.time()
    tempo_max_espera = 60 # segundos
    while True:
        if evento_abortar_seguranca and evento_abortar_seguranca.is_set():
            print("[DECOLAR] Abortagem de segurança ativa durante decolagem.")
            # Poderíamos tentar pousar aqui, mas para um aborto, é melhor apenas parar
            return False
        
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

# --- Placeholder de Visão Computacional ---
def detectar_alvo_visao():
    """
    PLACEHOLDER: Esta função simula a detecção de um alvo pela visão computacional.
    Você deve implementar a lógica real aqui.
    Retorna (alvo_encontrado, rel_norte, rel_leste, rel_baixo)
    rel_norte, rel_leste: posição relativa do alvo em metros em relação ao drone.
    rel_baixo: altitude relativa do alvo em metros em relação ao drone (positivo para baixo).
    """
    print("[VISAO] Simulando detecção de alvo...")
    time.sleep(2) # Simula o tempo de processamento da visão
    
    # Exemplo: Simula que o alvo foi encontrado 5m à frente, 0.5m à direita e 1m abaixo
    # Alvo será encontrado para teste após algumas tentativas de varredura
    if getattr(detectar_alvo_visao, 'contador', 0) < 3: # Encontra o alvo na 4ª chamada
        detectar_alvo_visao.contador = getattr(detectar_alvo_visao, 'contador', 0) + 1
        return False, 0, 0, 0
    
    print("[VISAO] Alvo detectado! Simulando posição relativa: 5m N, 0.5m E, 1m D.")
    return True, 5.0, 0.5, 1.0 # Exemplo de alvo encontrado: 5m à frente, 0.5m à direita, 1m abaixo
detectar_alvo_visao.contador = 0 # Inicializa o contador

def detectar_aruco_visao(id_aruco_alvo):
    """
    PLACEHOLDER: Esta função simula a detecção de um ArUco e do pacote correto.
    Você deve implementar a lógica real aqui.
    Retorna (aruco_encontrado, rel_norte, rel_leste, rel_baixo)
    """
    print(f"[VISAO_ARUCO] Simulando detecção do ArUco ID {id_aruco_alvo}...")
    time.sleep(3) # Simula o tempo de processamento
    
    # Exemplo: Simula que o ArUco foi encontrado 2m à frente, 0m à direita e 0.5m abaixo
    if getattr(detectar_aruco_visao, 'contador', 0) < 1: # Encontra o ArUco na 2ª chamada
        detectar_aruco_visao.contador = getattr(detectar_aruco_visao, 'contador', 0) + 1
        return False, 0, 0, 0

    print(f"[VISAO_ARUCO] ArUco ID {id_aruco_alvo} detectado! Simulando posição relativa: 2m N, 0m E, 0.5m D.")
    return True, 2.0, 0.0, 0.5 # Exemplo de ArUco encontrado
detectar_aruco_visao.contador = 0

# --- Lógica de Missão ---

def executar_padrao_busca(drone, area_busca_x, area_busca_y, altitude, velocidade_varredura=0.5):
    """
    Executa um padrão de varredura 'lawnmower' (cortador de grama) para cobrir uma área.
    area_busca_x: Largura da área a ser varrida no eixo X (leste-oeste, partindo do canto inferior esquerdo)
    area_busca_y: Comprimento da área a ser varrida no eixo Y (norte-sul, partindo do canto inferior esquerdo)
    altitude: Altitude de voo para a varredura
    velocidade_varredura: Velocidade de movimento durante a varredura (m/s)
    """
    print(f"\n[BUSCA] Iniciando padrão de varredura na área {area_busca_x}x{area_busca_y}m a {altitude}m de altitude...")
    
    # Supondo que a base de decolagem é (0,0) no canto inferior esquerdo.
    # O drone começa em (0,0) (HOME).
    # Vamos para o ponto inicial da varredura, por exemplo, o canto sul-oeste da área (0,0, -altitude)
    # Primeiro, subimos à altitude de busca (já feito pela decolagem)
    # Depois, para a varredura, movemos para frente e para os lados.

    # Movimento para frente e para os lados para cobrir a área
    x_atual = 0
    y_atual = 0
    # Inicialmente, movemos para o ponto mais à esquerda (0,0) da área de busca
    
    # Primeiro movemos para o início da primeira linha de varredura (ex: 0, 0)
    # O drone já está acima da base de decolagem, que é o (0,0) do sistema local.

    # Padrão de 'lawnmower'
    direcao = 1 # 1 para leste, -1 para oeste
    while y_atual < area_busca_y and not evento_abortar_seguranca.is_set():
        # Mover horizontalmente
        x_alvo = x_atual + direcao * area_busca_x
        print(f"  [BUSCA] Movendo de ({x_atual:.1f},{y_atual:.1f}) para ({x_alvo:.1f},{y_atual:.1f}) a {altitude:.1f}m")
        if not ir_para_posicao_local_ned_suave(x_alvo, y_atual, -altitude, velocidade_varredura, objeto_drone=drone):
            print("[BUSCA] Abortagem ou falha durante movimento horizontal.")
            return False, None
        
        # Simular detecção de alvo
        alvo_encontrado, rel_norte, rel_leste, rel_baixo = detectar_alvo_visao()
        if alvo_encontrado:
            print("[BUSCA] Alvo encontrado durante varredura!")
            return True, veiculo.location.global_relative_frame # Retorna localização atual do drone
        
        x_atual = x_alvo
        direcao *= -1 # Inverte a direção para a próxima linha

        # Mover para frente (North) para a próxima linha
        if y_atual + 1 < area_busca_y: # Move 1 metro para frente por linha
            y_alvo = y_atual + 1 # Pode ser ajustado para varrer mais densamente ou espaçado
            print(f"  [BUSCA] Movendo de ({x_atual:.1f},{y_atual:.1f}) para ({x_atual:.1f},{y_alvo:.1f}) a {altitude:.1f}m")
            if not ir_para_posicao_local_ned_suave(x_atual, y_alvo, -altitude, velocidade_varredura, objeto_drone=drone):
                print("[BUSCA] Abortagem ou falha durante movimento para próxima linha.")
                return False, None
            y_atual = y_alvo
        else:
            break # Fim da área Y
            
    if evento_abortar_seguranca.is_set():
        print("[BUSCA] Varredura interrompida por abortagem de segurança.")
        return False, None

    print("[BUSCA] Varredura completa, alvo não encontrado.")
    return False, None

def ir_para_posicao_local_ned_suave(norte, leste, baixo, velocidade_m_s, objeto_drone):
    if evento_abortar_seguranca.is_set():
        print("[IR_NED_SUAVE] Abortagem de segurança. Cancelando movimento.")
        return False
    
    posicao_atual_ned = objeto_drone.location.local_frame
    
    if not posicao_atual_ned:
        print("[IR_NED_SUAVE] Frame local NED não inicializado. Não é possível mover para posição local.")
    
        return False 
    norte_alvo = norte
    leste_alvo = leste
    baixo_alvo = baixo 

    print(f"  [IR_NED_SUAVE] Indo para Local NED: N={norte_alvo:.1f}, E={leste_alvo:.1f}, B={baixo_alvo:.1f} a {velocidade_m_s:.1f}m/s")
    def distancia_para_alvo_ned(atual, alvo_norte, alvo_leste, alvo_baixo):
        if not atual: return float('inf')
        delta_norte = alvo_norte - atual.north
        delta_leste = alvo_leste - atual.east
        delta_baixo = alvo_baixo - atual.down
        return math.sqrt(delta_norte**2 + delta_leste**2 + delta_baixo**2)

    distancia_restante = distancia_para_alvo_ned(posicao_atual_ned, norte_alvo, leste_alvo, baixo_alvo)

    while distancia_restante > 0.5 and not evento_abortar_seguranca.is_set():
        enviar_alvo_posicao_ned(norte_alvo, leste_alvo, baixo_alvo)
        time.sleep(1)

        posicao_atual_ned = objeto_drone.location.local_frame
        if not posicao_atual_ned:
            print("[IR_NED_SUAVE] Perdeu posição local NED. Abortando movimento.")
            return False
            
        distancia_restante = distancia_para_alvo_ned(posicao_atual_ned, norte_alvo, leste_alvo, baixo_alvo)
        print(f"    [IR_NED_SUAVE] Distância restante: {distancia_restante:.2f}m (N:{posicao_atual_ned.north:.1f}, E:{posicao_atual_ned.east:.1f}, B:{posicao_atual_ned.down:.1f})")

    if evento_abortar_seguranca.is_set():
        print("[IR_NED_SUAVE] Abortagem de segurança. Movimento interrompido.")
        return False
        
    print("[IR_NED_SUAVE] Posição Local NED alcançada.")
    return True

# --- Monitor de Abortagem de Segurança ---
def monitorar_abortagem_thread(evento):
    """
    Thread para monitorar a entrada do teclado para um comando de abortagem.
    Pressione 'q' e Enter para abortar.
    """
    print("\n[SEGURANCA] Monitoramento de abortagem ativado. Pressione 'q' + Enter a qualquer momento para abortar.")
    while not evento.is_set():
        try:
            caractere_entrada = input()
            if caractere_entrada.lower() == 'q':
                print("\n[SEGURANCA] Comando de abortagem 'q' recebido! Ativando evento de segurança.")
                evento.set()
                break
            else:
                print("[SEGURANCA] Comando inválido. Pressione 'q' para abortar.")
        except EOFError:
            print("[SEGURANCA] Entrada padrão fechada. Monitor de abortagem encerrado.")
            break
        except Exception as e:
            print(f"[SEGURANCA] Erro no monitor de abortagem: {e}")
            break


# --- Lógica Principal ---
def main():
    global veiculo

    print("Conectando ao drone na porta: %s" % STRING_CONEXAO)
    try:
        veiculo = connect(STRING_CONEXAO, wait_ready=True, baud=57600)
    except Exception as e:
        print(f"ERRO: Não foi possível conectar ao drone: {e}")
        return

    print("Drone conectado!")
    #print(f"Tipo do Veículo: {veiculo.system_type}")
    print(f"Versão do Firmware: {veiculo.version}")
    print(f"Modo padrão: {veiculo.mode.name}")
    print(f"Armado: {veiculo.armed}")

    # Inicia a thread de monitoramento de segurança
    thread_abortagem = threading.Thread(target=monitorar_abortagem_thread, args=(evento_abortar_seguranca,))
    thread_abortagem.daemon = True # Permite que o programa principal termine mesmo se a thread estiver rodando
    thread_abortagem.start()

    try:
        # --- Fase 0: Inicialização e Decolagem ---
        if not armar_drone_simplificado(veiculo, evento_abortar_seguranca):
            print("Falha ao armar. Abortando missão.")
            return
        
        if not decolar_drone_simplificado(veiculo, ALTITUDE_DECOLAGEM_PADRAO, evento_abortar_seguranca):
            print("Falha ao decolar. Abortando missão.")
            return
        
        # Após a decolagem, o drone deve estar em GUIDED e na altitude desejada.
        # Podemos pausar um pouco para estabilização
        print(f"Aguardando 5 segundos para estabilização na altitude {ALTITUDE_DECOLAGEM_PADRAO}m...")
        time.sleep(5)
        if evento_abortar_seguranca.is_set(): return # Verifica aborto

        # --- Missão 01: Bate e Volta ---
        print("\n=== INICIANDO MISSAO 01: BATE E VOLTA ===")
        # Edital: "A arena de competição consiste em uma área plana de 10 x 10 metros,
        # contendo diversas plataformas brancas com figuras geométricas distintas."
        # "O drone iniciará a missão a partir de uma base de decolagem, localizada no canto inferior esquerdo da arena."
        # Assumindo que o ponto de decolagem (HOME) é (0,0) do local NED.
        TAMANHO_ARENA_X = 10 # metros
        TAMANHO_ARENA_Y = 10 # metros

        local_alvo_encontrado = None
        alvo_encontrado, local_alvo_encontrado = executar_padrao_busca(
            veiculo, TAMANHO_ARENA_X, TAMANHO_ARENA_Y, ALTITUDE_DECOLAGEM_PADRAO
        )
        
        if evento_abortar_seguranca.is_set(): return # Verifica aborto

        if alvo_encontrado and local_alvo_encontrado:
            print("[MISSAO 01] Alvo da missão Bate e Volta encontrado!")
            # A visão computacional precisa guiar o drone para centralizar e pousar.
            # Aqui, simulamos o pouso na localização atual do drone após o "find".
            print("[MISSAO 01] Centralizando e pousando no alvo...")
            # Para pousar *no* alvo, idealmente você usaria um controlador de visão
            # para ajustar a posição relativa.
            # Como placeholder, podemos apenas pousar.
            # Se for um pouso de "precisão", a altitude final pode ser 0.5m e depois o pouso final.
            
            # Ajuste fino antes do pouso (simulado aqui)
            print("[MISSAO 01] Realizando ajuste fino sobre o alvo (visão computacional entraria aqui)...")
            # Ex: pequenos movimentos NED para centralizar.
            # enviar_velocidade_ned(0.1, 0, 0, 1) # Exemplo: 0.1m/s para frente por 1s

            if not pousar_drone_simplificado(veiculo, evento_abortar_seguranca):
                print("[MISSAO 01] Falha ao pousar no alvo. Abortando missão.")
                return
            
            # Após pousar no alvo, decolar novamente
            print("[MISSAO 01] Decolando novamente do alvo...")
            if not decolar_drone_simplificado(veiculo, ALTITUDE_DECOLAGEM_PADRAO, evento_abortar_seguranca):
                print("[MISSAO 01] Falha ao decolar do alvo. Abortando missão.")
                return
            
            # Retornar à base de decolagem (0,0, -altitude)
            print("[MISSAO 01] Retornando à base de decolagem (HOME)...")
            if not ir_para_posicao_local_ned_suave(0, 0, -ALTITUDE_DECOLAGEM_PADRAO, VELOCIDADE_CRUZEIRO_SOLO, objeto_drone=veiculo):
                print("[MISSAO 01] Falha ao retornar à base. Abortando missão.")
                return
        else:
            print("[MISSAO 01] Alvo da missão Bate e Volta NÃO encontrado na varredura.")
            # Se o alvo não foi encontrado, a missão "Bate e Volta" falha ou pode continuar para a próxima fase.
            # Para este exemplo, vamos considerar que a missão "Bate e Volta" foi concluída (mesmo sem encontrar).
            
        if evento_abortar_seguranca.is_set(): return 
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupção manual (Ctrl+C) detectada. Ativando abortagem de segurança.")
        evento_abortar_seguranca.set()
    except Exception as e:
        print(f"\n[MAIN] ERRO CRÍTICO na missão: {e}")
        import traceback
        traceback.print_exc()
        evento_abortar_seguranca.set() # Ativa a abortagem em caso de erro não tratado

    finally:
        # --- Fase Final: Retorno à Base e Pouso de Segurança ---
        print("\n=== FINALIZANDO MISSAO: RETORNANDO À BASE E POUSANDO ===")
        if veiculo and veiculo.armed:
            print("[FINALIZAR] Garantindo que o drone está em modo GUIDED para retorno e pouso.")
            veiculo.mode = VehicleMode("GUIDED")
            # Retorna para a posição de HOME (0,0) antes de pousar, se não estiver lá
            posicao_local_atual = veiculo.location.local_frame
            if posicao_local_atual and (abs(posicao_local_atual.north) > 1 or abs(posicao_local_atual.east) > 1):
                print("[FINALIZAR] Retornando à posição de HOME (0,0) antes do pouso final.")
                ir_para_posicao_local_ned_suave(0, 0, -ALTITUDE_DECOLAGEM_PADRAO, VELOCIDADE_CRUZEIRO_SOLO, objeto_drone=veiculo)
            
            pousar_drone_simplificado(veiculo, evento_abortar_seguranca)
        elif veiculo and not veiculo.armed:
            print("[FINALIZAR] Drone já desarmado ou não estava armado. Nenhuma ação de pouso necessária.")
        else:
            print("[FINALIZAR] Drone não conectado. Nenhuma ação de pouso necessária.")

        print("Fechando conexão com o drone.")
        if veiculo:
            veiculo.close()
        
        print("Fim da missão.")
        # Garante que a thread de abortagem termine
        evento_abortar_seguranca.set() # Set in case it wasn't already
        if thread_abortagem.is_alive():
            print("Aguardando thread de monitoramento de abortagem finalizar...")
            thread_abortagem.join(timeout=5) # Dá um tempo para a thread terminar
            if thread_abortagem.is_alive():
                print("Aviso: Thread de monitoramento de abortagem pode não ter terminado.")

if __name__ == "__main__":
    main()