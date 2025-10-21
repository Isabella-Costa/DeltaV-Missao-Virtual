import cv2 as cv
import numpy as np

class ShapeDetection:
    def __init__(self, epsilon_factor=0.02, toleracia_regular=0.15, min_area=500):
        self.epsilon_factor = epsilon_factor # epsilon dinamico para adaptar de acordo com a altura do drone
        self.toleracia_regular = toleracia_regular 
        self.min_area = min_area
        self.font = cv.FONT_HERSHEY_DUPLEX
        self.font_scale = 0.7
        self.font_thickness = 1
        self.text_colour = (0, 0, 255) 

    def get_shape_label(self, approx, contour):
        num_vertices = len(approx)
        area_contorno = cv.contourArea(contour)
        hull = cv.convexHull(contour)
        area_hull = cv.contourArea(hull)
        
        # Evita divisão por zero
        if area_hull == 0:
            return None
            
        solidez = area_contorno / float(area_hull)

        if num_vertices == 3 and solidez > 0.9 :
            return "Triangulo"
        elif num_vertices == 4 and solidez > 0.9:
            return "Quadrilatero"
        
        elif num_vertices == 5 and solidez > 0.9:
            len_lados = []
            for i in range(5):
                p1 = approx[i][0]
                p2 = approx[(i + 1) % 5][0]
                distancia = np.linalg.norm(p1 - p2)
                len_lados.append(distancia)
            
            min_len = min(len_lados)
            max_len = max(len_lados)

            # Se a razão entre o maior e o menor lado for pequena é regular
            if min_len > 0 and (max_len / min_len) < (1 + self.toleracia_regular ):
                return "Pentagono"
            else:
                return "Casa"
            
        elif num_vertices == 6 and solidez < 0.75:
            return "Hexagono"
        elif num_vertices == 10 and  solidez < 0.8: 
            return "Estrela"
        elif num_vertices <= 13 and solidez < 0.75:
            return "Cruz"
        elif num_vertices > 13 and solidez > 0.95: 
            return "Circulo"
        else:
            return None
        
    def detecta_contorno(self, frame):
        shapes_detectados = []
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # Converte para escala cinza
        thresh = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2) #binarização
        # cv.ADAPTIVE_THRESH_GAUSSIAN_C : o valor limite é uma soma ponderada gaussiana dos valores de vizinhança menos a constante C
        # cv.THRESH_BINARY_INV 

        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE) 

        for contour in contours:
            if cv.contourArea(contour) < self.min_area:
                continue
               
            #aproximação do contorno
            epsilon = self.epsilon_factor * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)
            label = self.get_shape_label(approx, contour)

            # pega os momentos da forma dectada
            if label:
                M = cv.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    shape_data = {'label': label, 'center': (cx, cy), 'contour': approx}
                    shapes_detectados.append(shape_data)
        
        return shapes_detectados
    
    def draw_contorno(self, frame, shapes_detectados):
        font = cv.FONT_HERSHEY_DUPLEX
        for shape in shapes_detectados:
            cv.drawContours(frame, [shape['contour']], -1, (0, 255, 0), 3)
            cv.putText(frame, shape['label'], (shape['center'][0] - 40, shape['center'][1]), font, 0.7, (0, 0, 255), 1)
        return frame


