import cv2 as cv
import numpy as np
import time

#KNOWN_WIDTH_CM = 8.5
KNOWN_WIDTH_CM = 30

#FOCAL_LENGTH_PIXELS = 451.76
FOCAL_LENGTH_PIXELS = 476

# F = (P * D) / W
# F = (128 * 30) / 8.5 = 451.76
# P = Largura em pixels | D = Distancia(cm) | W = Largura real

def detect_squares_in_frame(frame, center_x, center_y):
    # ETAPA DE PRÉ-PROCESSAMENTO ROBUSTO
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (5, 5), 0)
    binary_image = cv.adaptiveThreshold(
        blurred, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv.THRESH_BINARY_INV,
        21, 5
    )

    # Opcional: Fechamento morfológico
    #kernel = np.ones((5, 5), np.uint8)
    #binary_image = cv.morphologyEx(binary_image, cv.MORPH_CLOSE, kernel)

    contours, _ = cv.findContours(binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contour_frame = cv.cvtColor(binary_image, cv.COLOR_GRAY2BGR)

    #figura_desejada = 'quadrado'
    found_figure = False
    figura = ''
    figure_contour = None
    error_x = 0
    error_y = 0
    distance_cm = 0

    if contours:
        # Pega o maior contorno para evitar processar ruídos menores
        largest_contour = max(contours, key=cv.contourArea)
        if cv.contourArea(largest_contour) > 500: # Filtro de área mínima
            epsilon = 0.05 * cv.arcLength(largest_contour, True)
            approx = cv.approxPolyDP(largest_contour, epsilon, True)

            #Centroid
            M = cv.moments(approx)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            
            #1 TENTA DETECTAR UM QUADRADO (4 VÉRTICES)
            if len(approx) == 4 :
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
            elif len(approx) == 5 :
                rect = cv.minAreaRect(approx)
                (x, y), (w, h), ang = rect
                
                contour_area = cv.contourArea(approx)
                rect_area = w * h
                solidity = contour_area / rect_area if rect_area > 0 else 0
                
                if solidity > 0.6:
                    found_figure = True
                    figura = 'Pentagono'
                    figure_contour = approx
                    square_center_x = cX
                    square_center_y = cY
            
            #3 TENTA DETECTAR UM TRIÂNGULO (3 VÉRTICES)
            elif len(approx) == 3 :
                rect = cv.minAreaRect(approx)
                (x, y), (w, h), ang = rect
                
                contour_area = cv.contourArea(approx)
                rect_area = w * h
                solidity = contour_area / rect_area if rect_area > 0 else 0
                
                if solidity > 0.3:
                    found_figure = True
                    figura = 'Triangulo'
                    figure_contour = approx
                    square_center_x = cX
                    square_center_y = cY
        
        if found_figure:
            try:
                # Desenha na janela principal
                cv.drawContours(frame, [figure_contour], 0, (0, 255, 0), 2)
                
                # Desenha na janela de debug
                cv.drawContours(contour_frame, [figure_contour], 0, (0, 255, 0), 2)
            except cv.error as e:
                # Opcional: imprime um aviso no console para saber que um erro de desenho ocorreu
                print(f"Aviso: erro ao desenhar contorno ignorado - {e}")

            # DESENHAR A RETÍCULA
            cv.line(frame, (square_center_x - 10, square_center_y), (square_center_x + 10, square_center_y), (0, 255, 0), 1)
            cv.line(frame, (square_center_x, square_center_y - 5), (square_center_x, square_center_y + 5), (0, 255, 0), 1)

            # DESENHA O TEXTO
            text_x = square_center_x - 40 # Ajuste para centralizar o texto
            text_y = square_center_y - 70# Ajuste para posicionar acima da figura
            cv.putText(frame, figura, (text_x, text_y), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                
            # Desenha a linha do vetor de erro
            cv.line(frame, (center_x, center_y), (square_center_x, square_center_y), (255, 255, 0), 2)
               
    cv.imshow('Contours',contour_frame)
    return frame, error_x, error_y, found_figure, distance_cm

def main():
    cap = cv.VideoCapture(1)
    if not cap.isOpened():
        print("Erro: Não foi possível acessar a câmera.")
        return

    print("Pressione 'q' para sair.")

    # Variáveis para armazenar o último erro conhecido
    last_known_error_x, last_known_error_y, last_known_distance = 0,0,0
    mission_status = 'PROCURANDO ALVO'
    aprox_final_tempo = 0
    aprox_final_alt = 40

    # Parâmetros da retícula central
    crosshair_size = 15
    crosshair_color = (0, 0, 255)
    crosshair_thickness = 2

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Obter as dimensões do quadro
        height, width, _ = frame.shape
        center_x = int(width / 2)-80
        center_y = int(height / 2)

        cv.line(frame, (center_x - crosshair_size, center_y), (center_x + crosshair_size, center_y), crosshair_color, crosshair_thickness)
        cv.line(frame, (center_x, center_y - crosshair_size), (center_x, center_y + crosshair_size), crosshair_color, crosshair_thickness)

        processed_frame, new_error_x, new_error_y, found, new_distance= detect_squares_in_frame(frame, center_x, center_y)
        
        # MÁQUINA DE ESTADOS
        # Verifica se há quadrado e se não está na aproximação final
        if mission_status not in ['APROX. FINAL', 'POUSO REALIZADO']:
            if found:
                last_known_error_x = new_error_x
                last_known_error_y = new_error_y
                last_known_distance = new_distance
                text_color = (0, 255, 0) # Mudar a cor com a detecção
                mission_status = 'CENTRALIZANDO'

                # Transição para descida (usando abs para tratar erro à esquerda/direita)
                if abs(last_known_error_x) < 20 and abs(last_known_error_y) < 20:
                    mission_status = 'INICIANDO DESCIDA'

                if mission_status == 'INICIANDO DESCIDA' and last_known_distance < aprox_final_alt:
                    mission_status = 'APROX. FINAL'
                    aprox_final_tempo = time.time() # Inicia o timer
                    
            else:
                text_color = (0, 0, 255) # Mudar a cor sem a detecção
                mission_status = 'PROCURANDO ALVO'

        # Verifica a aproximação final
        if(mission_status == 'APROX. FINAL'):
            if(time.time() - aprox_final_tempo > 3.0):
                mission_status = 'POUSO REALIZADO'

        if(mission_status == 'POUSO REALIZADO'):
            text_color = (255, 255, 255)


        text_error_x = f"Erro X: {last_known_error_x:04d} px"
        text_error_y = f"Erro Y: {last_known_error_y:04d} px"
        text_distance = f"Altitude: {last_known_distance:.2f} cm"
        cv.putText(processed_frame, text_error_x, (20, 25), cv.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
        cv.putText(processed_frame, text_error_y, (20, 50), cv.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
        cv.putText(processed_frame, text_distance, (220, 25), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv.putText(processed_frame, f"Status: {mission_status}", (220, 50), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        cv.imshow('Missão Autônoma', processed_frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()