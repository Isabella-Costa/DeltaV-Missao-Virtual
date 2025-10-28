import time
import math
import threading
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import funcoes_controle.py
# --- Configurações Globais---
STRING_CONEXAO = "udp:127.0.0.1:14550" 
drone=STRING_CONEXÃO
estado="armando"

# ----Armando----
if estado== "armando":
  print("Drone sendo armado")
  armar(drone):
  if drone.armed ==True:
    estado="decolando"
# ----Decolando----
elif estado=="decolando":
  print("Drone Decolando para uma altitude de 3 metros")
  decolar(drone,3)
  if decolar(drone,3)==True:
    estado="vasculhar"
# ----vasculhando-----
elif estado=="vasculhar":
  if figura==True
    estado=="centralizando"
  for i in range(9)
    if i % 2 ==0 and figura!=True:
      velocidade(0,-1,0,9)
      print("drone se deslocando para a esquerda em 1 metro por segundo por 9 segundos")
      velocidade(1,0,0,1)
      print("drone se deslocando para frente em 1 metro por segundo por 1 segundo")
    elif i % 2 !=0 and figura !=True:
      velocidade(0,1,0,9)
      print("drone se deslocando para a direita em 1 metro por segundo por 9 segundos")
      velocidade(1,0,0,1)
      print("drone se deslocando para frente em 1 metro por segundo por 1 segundo")
# ----Centralizando----
elif estado=="centralizando"

    
