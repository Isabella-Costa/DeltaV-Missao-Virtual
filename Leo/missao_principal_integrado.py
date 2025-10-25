import cv2 as cv
import numpy as np
import time

from nova_visao import detect_target
from controle_simplificado import (connect, armar_drone_simplificado, 
                            decolar_drone_simplificado, velocidade_enviada_ned, 
                            pousar_drone_simplificado, safety_abort, vehicle, 
                            CONNECTION_STRING, threading)

