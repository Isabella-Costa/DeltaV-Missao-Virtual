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
if estado=="decolando":
  decolar(drone,3)
