import cv2
import numpy as np 
from config import PRE_PROCESSAMENTO, FILTRAGEM_CONTORNOS, DESENHO, ANGULACAO, ANGULACAO_PENTAGONO_REG, FILTRO_KALMAN, SOLIDEZ

def incializar_kalman():
    kf = cv2.KalmanFilter(4, 2)

    kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)
    
    kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)
    
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * FILTRO_KALMAN["uncertainty_magnitude"]

    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * FILTRO_KALMAN["noise_magnitude"]
    return kf
    

def detectar_formas(frame):
    
    frame_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # conversão para escala de cinza
    clahe= cv2.createCLAHE(PRE_PROCESSAMENTO["clahe_cliplimit"], PRE_PROCESSAMENTO["clahe_grid_size"])
    frame_clahe = clahe.apply(frame_cinza) #Aplica contraste local sem estourar áreas mais claras das quais podem influenciar negativamente na detecção
    frame_suave = cv2.GaussianBlur(frame_clahe, PRE_PROCESSAMENTO["gaussian_blur_ksize"], 0)   # desfoque gaussiano                                       
    bordas_canny = cv2.Canny( frame_suave, 50, 100)
    

    contornos, hierarquia = cv2.findContours(bordas_canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # RETR_TREE → retorna contornos + hierarquia

    espectro_ratio = 0
    quadrados_internos = []
    triangulos_internos = []
    pentagonos_regulares = []
    pentagonos_irregulares = []
    estrela = []
    cruz = []
    circulo = []

    if hierarquia is not None:
        hierarquia = hierarquia[0]
    
        def angulo_cos(p1, p2, p3):
            v1 = p1 - p2
            v2 = p3 - p2
            cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            cosang = np.clip(cosang, -1.0, 1.0)  # Garante que o valor esteja no intervalo válido para arccos
            return np.degrees(np.arccos(cosang))
        
        
        for i, cnt in enumerate(contornos):

            area = cv2.contourArea(cnt)
            perimetro = cv2.arcLength(cnt, True)
            epsilon =  FILTRAGEM_CONTORNOS["contour_epsilon"] * perimetro #distância máxima do contorno ao contorno aproximado
            approx = cv2.approxPolyDP(cnt, epsilon , True)
            num_vertices = len(approx)


            # Filtro de quadrado
            if len(approx) == 4 and area > FILTRAGEM_CONTORNOS["min_area"]: # área em pixels, calcular qual valor seria ideal 
                print(area) 
                angulos = []
                for j in range(4):
                    p1 = approx[j][0]
                    p2 = approx[(j+1)%4][0]
                    p3 = approx[(j+2)%4][0]
                    angulos.append(angulo_cos(p1, p2, p3))


                if all ( ANGULACAO["lower_limit"] <= ang <= ANGULACAO["upper_limit"] for ang in angulos): #Tolera 10°
                    rect = cv2.minAreaRect(approx)
                    (w, h) = rect[1]

                    if h != 0:
                        espectro_ratio = float(w) / h if w > h else float(h) / w

                        if FILTRAGEM_CONTORNOS["min_aspect_ratio"] <= espectro_ratio <= FILTRAGEM_CONTORNOS["max_aspect_ratio"]:
                            # acessa hierarquia do contorno atual
                            # hierarquia~[0][i] = [next, prev, first_child, parent]
                            parent = hierarquia[i][3]
                            child = hierarquia[i][2]

                            # ex: Se não tem pai -> quadrado externo
                            if parent != -1 and child == -1:
                                quadrados_internos.append((area, approx))
            elif num_vertices == 3 and area > FILTRAGEM_CONTORNOS["min_area"]: #triangulo
                 parent = hierarquia[i][3]
                 child = hierarquia[i][2]

                 if parent != -1 and child == -1:
                    triangulos_internos.append((area, approx))
            
            elif num_vertices == 5 and area > FILTRAGEM_CONTORNOS["min_area"]: #pentagono regular
                angulos = []
                for j in range (5):
                      p1 = approx[j][0]
                      p2 = approx[(j+1)%5][0]
                      p3 = approx[(j+2)%5][0]
                      angulos.append(angulo_cos(p1, p2, p3))

                if all (ANGULACAO_PENTAGONO_REG["lower_limit"] <= ang <= ANGULACAO_PENTAGONO_REG["upper_limit"] for ang in angulos):
                            rect = cv2.minAreaRect(approx)
                            (w, h) = rect[1]

                            if h != 0:
                                espectro_ratio = float(w) / h if w > h else float(h) / w

                                if FILTRAGEM_CONTORNOS["min_aspect_ratio"] <= espectro_ratio <= FILTRAGEM_CONTORNOS["max_aspect_ratio"]:
                                     pentagonos_regulares.append((area, approx))
                else:
                             pentagonos_irregulares.append((area, approx)) #pentagono irregular
           # Estrela
            if len(cnt) > 20:
                hull_indices = cv2.convexHull(cnt, returnPoints=False)
                if hull_indices is not None and len(hull_indices) > 3:
                    defects = cv2.convexityDefects(cnt, hull_indices)

                    if defects is not None:
                        num_defects = defects.shape[0]
            if num_defects == 8:
                hull_area = cv2.contourArea(cv2.convexHull(cnt))
                if hull_area > 0:
                    solidez = float(area) / hull_area

                    if solidez < SOLIDEZ["solidez_cross"]:
                        cruz.append((area, cnt))
                        continue
            if perimetro > 0:
                circularidade = (4 * np.pi * area) / (perimetro * perimetro)

                if circularidade > SOLIDEZ["circularidade_min"]:
                    circulo.append((area, cnt))
                    continue           
                            
    listas_formas = {
         "quadrados": quadrados_internos,
         "triangulos": triangulos_internos,
         "pent_reg": pentagonos_regulares,
         "pent_irr": pentagonos_irregulares,
         "estrela": estrela,
         "cruz": cruz,
         "circulo": circulo,

    }
    return quadrados_internos, triangulos_internos, pentagonos_regulares, pentagonos_irregulares, estrela, cruz, circulo, bordas_canny, frame_clahe

    
         

def desenhar(frame, quadrados):
    for area, quadrado in quadrados:
        cv2.drawContours(frame, [quadrado], -1, (255,0,0), 3 )

        x, y, _, _ = cv2.boundingRect(quadrado)
        cv2.putText(frame, F"Quadrado interno - area:{int(area)}", (x, y - 10), DESENHO["font"], 0.7, DESENHO["cor_quadrado"], 2)

         # Cálculo e desenho do centro do quadrado
 
        M = cv2.moments(quadrado) # retorna um dicionário com vários momentos
        if M["m00"] != 0: # m00 representa a área do contorno
                    cx = int(M["m10"]/M["m00"]) # m10 soma de todas as coord x 
                    cy = int(M["m01"]/M["m00"]) # m01 soma de todas as coord y 
                    cv2.circle(frame, (cx, cy), 5, DESENHO["cor_centro"], -1)

def desenhar_triangulos(frame, triangulos): #Função de desenho dos contornos do triangulo
     for area, triangulo in triangulos:
          cv2.drawContours(frame, [triangulo], -1, (0, 0, 255), 3) #Cor vermelha em BGR

          x, y, _, _ = cv2.boundingRect(triangulo)
          cv2.putText(frame, F"Triangulo - area {int (area)}", (x, y - 10), DESENHO["font"], 0.7, (0, 0, 255), 2)

          M = cv2.moments(triangulo)
          if M["m00"] != 0:
               cx = int (M["m10"] /M["m00"])
               cy = int (M["m01"] / M["m00"])
               cv2.circle(frame, (cx, cy), 5, DESENHO["cor_centro"], -1)

def desenhar_pentagonos_reg (frame, pentagonos):
     for area, pentagono in pentagonos:
         cv2.drawContours(frame, [pentagono], -1, (0, 255, 0), 3) 
         x, y, _, _ = cv2.boundingRect(pentagono)
         cv2.putText(frame, F"Pentagono regular - area {int (area)}", (x, y - 10), DESENHO["font"], 0.7, (0, 255, 0), 2)
         M = cv2.moments(pentagono)
         if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 5, DESENHO["cor_centro"], -1)

def desenhar_pentagonos_irr (frame, pentagonos):
     for area, pentagono in pentagonos:
          cv2.drawContours(frame, [pentagono], -1, (255, 0, 255), 3) #Cor magenta em BGR

          x, y, _, _ = cv2.boundingRect(pentagono)
          cv2.putText(frame, F"Pentagono irregular - area {int (area)}", (x, y - 10), DESENHO["font"], 0.7, (255, 0, 255), 2)

          M = cv2.moments(pentagono)
          if M["m00"] != 0:
               cx = int (M["m10"] / M["m00"])
               cy = int (M["m01"] / M["m00"])
               cv2.circle(frame, (cx, cy), 5, DESENHO["cor_centro"], -1)
def desenhar_estrela (frame, estrelas):
     for area, estrela in estrelas:
          cv2.drawContours(frame, [estrela], -1, (0, 255, 255), 3) #Cor amarela em BGR

          x, y, _, _ = cv2.boundingRect(estrela)
          cv2.putText(frame, F"Estrela - area {int (area)}", (x, y - 10), DESENHO["font"], 0.7, (0, 255, 255), 2)

          M = cv2.moments(estrela)
          if M["m00"] != 0:
               cx = int (M["m10"] / M["m00"])
               cy = int (M["m01"] / M["m00"])
               cv2.circle(frame, (cx, cy), 5, DESENHO["cor_centro"], -1)
def desenhar_cruz (frame, cruzes):
     for area, cruz in cruzes:
         cv2.drawContours(frame, [cruz], -1, (128, 0, 128), 3) 
         x, y, _, _ = cv2.boundingRect(cruz)
         cv2.putText(frame, F"Cruz - area {int (area)}", (x, y - 10), DESENHO["font"], 0.7, (128, 0, 128), 2)
         M = cv2.moments(cruz)
         if M["m00"] != 0:
                # --- CORRIGIDO ---
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 5, DESENHO["cor_centro"], -1)
def desenhar_circulo (frame, circulos):
     for area, circulo in circulos:
         # Para círculos, é melhor usar minEnclosingCircle para desenhar
         (x, y), raio = cv2.minEnclosingCircle(circulo)
         centro = (int(x), int(y))
         raio = int(raio)
         
         cv2.circle(frame, centro, raio, (0, 128, 128), 3) # Desenha a borda
         
         x_texto, y_texto, _, _ = cv2.boundingRect(circulo)
         cv2.putText(frame, F"Circulo - area {int (area)}", (x_texto, y_texto - 10), DESENHO["font"], 0.7, (0, 128, 128), 2)

         # Usa o 'centro' calculado que já é (int, int)
         cv2.circle(frame, centro, 5, DESENHO["cor_centro"], -1)

