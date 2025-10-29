import time
import math
import threading
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from funcoes_controle.py import armar, decolar, pousar, distancia_metros
# --- Configurações Globais---
STRING_CONEXAO = "udp:127.0.0.1:14550" 
drone=STRING_CONEXÃO
estado="armando"

# ----Armando----
if estado == "armando":
  print("Drone sendo armado")
  armar(drone):
  if drone.armed == True:
    estado="decolando"
# ----Decolando----
elif estado=="decolando":
  print("Drone Decolando para uma altitude de 3 metros")
  decolar(drone,3)
  if decolar(drone,3)==True:
    estado="andando"
#a-------andando------
elif estado=='andando':
  print('Deslocando o drone em 2 metro para frente')
  velocidade(2,0,0,1)
  estado='pousando'

# ----pousando-----
elif estado=="pousando":
  print("Drone pousando")
  pousar(drone)
  if pousar(drone)==True:
    estado="decolar1"
# ----decolando1----
elif estado=="decolar1":
  print("Decolando o drone")
  decolar(drone,3)
  if decolar(drone,3)==True:
    estado="rtl"
# ----voltando para casa------
elif estado=="rtl":
  print("voltando par casa")
  while not vehicle.mode.name == "RTL":
    print(" Aguardando a mudança de modo...")
    time.sleep(1)
    print("Modo alterado para RTL com sucesso!")
    print("O veículo agora está retornando e pousando...")
    print("Aguardando o pouso...")
    vehicle.close()
    print("Conexão fechada.")

    
  
