import cv2 as cv
import numpy as np
import time

def detect_target(frame, center_x, center_y):
    # ETAPA DE PRÉ-PROCESSAMENTO ROBUSTO
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (5, 5), 0)
    binary_image = cv.adaptiveThreshold(
        blurred, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv.THRESH_BINARY_INV,
        21, 5
    )
    contours, _ = cv.findContours(binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contour_frame = cv.cvtColor(binary_image, cv.COLOR_GRAY2BGR)
    detection_data = {
        'found_figure': False,
        'figura': '',
        'figure_contour': None,
        'rect': None,
        'error_x': 0,
        'error_y': 0,
        'distance_cm': 0
    }
    figura_desejada = 'quadrado'

    # Detecção das figuras
    if contours:
        # Pega o maior contorno para evitar processar ruídos menores
        largest_contour = max(contours, key=cv.contourArea)
        if cv.contourArea(largest_contour) > 200: # Filtro de área mínima
            epsilon = 0.03 * cv.arcLength(largest_contour, True)
            approx = cv.approxPolyDP(largest_contour, epsilon, True)

            #Centroid
            M = cv.moments(approx)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            
            #1 TENTA DETECTAR UM QUADRADO (4 VÉRTICES)
            if len(approx) == 4 and figura_desejada == 'quadrado':
                rect = cv.minAreaRect(approx)
                (x, y), (w, h), ang = rect

                if w < h: w, h = h, w
                aspect_ratio = float(w) / h if h > 0 else 0
                contour_area = cv.contourArea(approx)
                rect_area = w * h
                solidity = contour_area / rect_area if rect_area > 0 else 0

                if (0.85 <= aspect_ratio <= 1.2) and (solidity > 0.85):
                    found_figure = True
                    figura = 'Quadrado'
                    figure_contour = approx
                    square_center_x = cX
                    square_center_y = cY
        
            #2 TENTA DETECTAR UM PENTÁGONO (5 VÉRTICES)
            elif len(approx) == 5 and (figura_desejada == 'pentagono' or figura_desejada == 'casa'):
                rect = cv.minAreaRect(approx)
                (x, y), (w, h), ang = rect
                
                contour_area = cv.contourArea(approx)
                rect_area = w * h
                solidity = contour_area / rect_area if rect_area > 0 else 0
                
                if solidity > 0.6:
                    side_lengths = []

                    for i in range(len(approx)):
                        # Ponto de início e ponto final do segmento
                        p1 = approx[i][0]
                        p2 = approx[(i + 1) % len(approx)][0]
                        
                        # Calcula a distância euclidiana entre os dois pontos (comprimento do lado)
                        # np.linalg.norm é um jeito eficiente de calcular a distância
                        length = np.linalg.norm(p1 - p2)
                        side_lengths.append(length)
                    
                    # Encontra o maior e o menor lado
                    min_side = min(side_lengths)
                    max_side = max(side_lengths)
                    
                    # A razão é: Maior Lado / Menor Lado
                    # Adicionamos uma pequena constante (epsilon) para evitar divisão por zero,
                    # embora min_side não deva ser zero em um contorno válido.
                    side_ratio = max_side / (min_side if min_side > 0 else 1)
                    THRESHOLD_REGULARITY = 2 # Ajuste este valor conforme seus testes
                    if side_ratio < THRESHOLD_REGULARITY:
                        figura = "Pentagono"
                    else:
                        figura= "Casa"
                    found_figure = True
                    figure_contour = approx
                    square_center_x = cX
                    square_center_y = cY
            
            #3 TENTA DETECTAR UM TRIÂNGULO (3 VÉRTICES)
            elif len(approx) == 3 and figura_desejada == 'triangulo':
                rect = cv.minAreaRect(approx)
                (x, y), (w, h), ang = rect
                
                contour_area = cv.contourArea(approx)
                rect_area = w * h
                solidity = contour_area / rect_area if rect_area > 0 else 0
                
                if solidity > 0.5:
                    found_figure = True
                    figura = 'Triangulo'
                    figure_contour = approx
                    square_center_x = cX
                    square_center_y = cY

            #4 TENTA DETECTAR UMA ESTRELA (10 VÉRTICES)
            elif figura_desejada == 'estrela':
                # Obtém o retângulo delimitador não rotacionado (Upright Bounding Box)
                x, y, w, h = cv.boundingRect(approx) 
                bounding_area = w * h
                contour_area = cv.contourArea(approx)

                extent_ratio = contour_area / bounding_area if bounding_area > 0 else 0
                # Uma estrela de cinco pontas regular preenche aproximadamente 38.2% do seu bounding box.

                # O Convex Hull é o menor polígono convexo que contém a estrela.

                hull = cv.convexHull(approx)
                hull_area = cv.contourArea(hull)
                convexity_ratio = contour_area / hull_area if hull_area > 0 else 0

                if (0.35 <= extent_ratio <= 0.45) and (0.55 <= convexity_ratio <= 0.65): # Analisa a razão de Extensão/Convexidade
                    found_figure = True
                    figura = 'Estrela'
                    figure_contour = approx
                    square_center_x = cX
                    square_center_y = cY
                    rect = cv.minAreaRect(approx)

            #5 TENTA DETECTAR UMA CRUZ (12 VÉRTICES)
            elif len(approx) >= 8 and len(approx) <=16 and figura_desejada == 'cruz':
                contour_area = cv.contourArea(approx)
                hull = cv.convexHull(approx)
                hull_area = cv.contourArea(hull)
                solidity = contour_area / hull_area if hull_area > 0 else 0
                is_solid_enough = (solidity >= 0.45 and solidity <= 0.80)
                #if w < h: w, h = h, w
                #aspect_ratio = w / h if h > 0 else 0
                #is_aspect_ratio_ok = (aspect_ratio >= 0.8 and aspect_ratio <= 1.2) # com margem
                # Verificação da estrutura de 4 braços
                hull_indices = cv.convexHull(approx, returnPoints=False)
                num_defects = 0
                if hull_indices is not None and len(hull_indices) > 3:
                    try:
                        defects = cv.convexityDefects(approx, hull_indices)
                        if defects is not None: num_defects = len(defects)
                    except cv.error as e:
                        num_defects = 0
                is_correct_defects = (num_defects >= 3 and num_defects <= 5) # esperamos 4 defeitos

                if is_solid_enough and is_correct_defects:
                    found_figure = True
                    figura = 'Cruz'
                    figure_contour = approx
                    square_center_x = cX
                    square_center_y = cY
                    rect = rect

    return detection_data