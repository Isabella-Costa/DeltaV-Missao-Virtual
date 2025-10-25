import cv2
import numpy as np
from config import PRE_PROCESSAMENTO, FILTRAGEM_CONTORNOS, DESENHO, ANGULACAO, ANGULACAO_PENTAGONO_REG, FILTRO_KALMAN, SOLIDEZ

# FUNÇÃO KALMAN 
def incializar_kalman():
    kf = cv2.KalmanFilter(4, 2)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * FILTRO_KALMAN.get("uncertainty_magnitude", 1e-4)
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * FILTRO_KALMAN.get("noise_magnitude", 1e-1)
    return kf


# DETECÇÃO DE FORMAS 
class ShapeDetector:

    def __init__(self):
        # Pre-processamento
        self.clahe_cliplimit = PRE_PROCESSAMENTO.get("clahe_cliplimit", 2.0)
        self.clahe_grid_size = PRE_PROCESSAMENTO.get("clahe_grid_size", (8, 8))
        self.blur_ksize = PRE_PROCESSAMENTO.get("gaussian_blur_ksize", (5, 5))
        self.canny_1 = PRE_PROCESSAMENTO.get("canny_thresh_1", 50)
        self.canny_2 = PRE_PROCESSAMENTO.get("canny_thresh_2", 100)
        
        # Filtragem
        self.min_area = FILTRAGEM_CONTORNOS.get("min_area", 500)
        self.epsilon_factor = FILTRAGEM_CONTORNOS.get("contour_epsilon", 0.009)
        self.min_aspect_ratio = FILTRAGEM_CONTORNOS.get("min_aspect_ratio", 0.8)
        self.max_aspect_ratio = FILTRAGEM_CONTORNOS.get("max_aspect_ratio", 1.2)

        # Angulação
        self.quad_angle_min = ANGULACAO.get("lower_limit", 80)
        self.quad_angle_max = ANGULACAO.get("upper_limit", 100)
        self.pent_angle_min = ANGULACAO_PENTAGONO_REG.get("lower_limit", 100)
        self.pent_angle_max = ANGULACAO_PENTAGONO_REG.get("upper_limit", 116)

        #  Solidez
        self.solidez_star_min = SOLIDEZ.get("star_min", 0.45)
        self.solidez_star_max = SOLIDEZ.get("star_max", 0.6)
        self.solidez_cross_min = SOLIDEZ.get("cross_min", 0.55)
        self.solidez_cross_max = SOLIDEZ.get("cross_max", 0.7)
        self.circularity_min = SOLIDEZ.get("circularidade_min", 0.85)

        # Desenho
        self.font = DESENHO.get("font", cv2.FONT_HERSHEY_SIMPLEX)
        self.font_scale = DESENHO.get("font_scale", 0.7)
        self.font_thickness = DESENHO.get("font_thickness", 2)
        self.center_color = DESENHO.get("cor_centro", (0, 0, 255))
        
        # Mapeamento de cores para desenho
        self.color_map = {
            "Quadrilatero": DESENHO.get("cor_quadrado", (255, 0, 0)),
            "Triangulo": (0, 0, 255),
            "Pentagono": (0, 255, 0),
            "Casa": (255, 0, 255),
            "Cruz": (128, 0, 128),
            "Estrela": (0, 255, 255),
            "Circulo": (0, 128, 128)
        }

    def angulo_cos(self, p1, p2, p3):
        v1 = p1 - p2
        v2 = p3 - p2

        # np.linalg.norm calcula a magnitude (comprimento) do vetor
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        if norm_v1 == 0 or norm_v2 == 0:
            return 0 # Evita divisão por zero
        
        cosang = np.dot(v1, v2) / (norm_v1 * norm_v2)
        cosang = np.clip(cosang, -1.0, 1.0) # Garantir que o valor esteja no intervalo
        return np.degrees(np.arccos(cosang))

    def detect(self, frame):
            # Pré-processamento
            frame_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(self.clahe_cliplimit, self.clahe_grid_size)
            frame_clahe = clahe.apply(frame_cinza)
            frame_suave = cv2.GaussianBlur(frame_clahe, self.blur_ksize, 0)
            bordas_canny = cv2.Canny(frame_suave, self.canny_1, self.canny_2)

            # Contornos
            contornos, hierarquia = cv2.findContours(bordas_canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            shapes_detectados = [] # Lista unificada de formas

            if hierarquia is None:
                return [], bordas_canny, frame_clahe

            hierarquia = hierarquia[0] # Pega o primeiro nível da hierarquia
            
            for i, cnt in enumerate(contornos):
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue

                perimetro = cv2.arcLength(cnt, True)
                
                # Epsilon (precisão) 
                epsilon = self.epsilon_factor * perimetro 
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                num_vertices = len(approx)

                # Calcula o centro 
                M = cv2.moments(cnt)
                center = None
                if M["m00"] != 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                else:
                    continue # Ignora formas sem área/centro

                parent = hierarquia[i][3]
                child = hierarquia[i][2]

                shape_data = {'center': center, 'area': area, 'label': 'Desconhecido'}

                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                if hull_area == 0: continue
                solidez = float(area) / hull_area

                # Concavos -------------------------------------------------------------------------------------------------------------------------------------------------
                if len(cnt) > 20:
                    hull_indices = cv2.convexHull(cnt, returnPoints=False)
                    if hull_indices is not None and len(hull_indices) > 3:
                        try:
                            defects = cv2.convexityDefects(cnt, hull_indices)
                            if defects is not None:
                                num_defects = defects.shape[0]

                                #print(f"Forma Côncava: Defeitos={num_defects}, Solidez={solidez:.2f}")

                                if (4 <= num_defects <= 10):
                                    # ESTRELA
                                    if self.solidez_star_min < solidez < self.solidez_star_max:
                                        shape_data.update({'label': 'Estrela', 'contour': cnt})
                                        shapes_detectados.append(shape_data)
                                        continue
                                        
                                    # CRUZ
                                    if  self.solidez_cross_min < solidez < self.solidez_cross_max:
                                        shape_data.update({'label': 'Cruz', 'contour': cnt})
                                        shapes_detectados.append(shape_data)
                                        continue

                        except cv2.error as e:
                            pass # Ignora contornos malformados

                    # Círculo 
                    if perimetro > 0:
                        circularidade = (4 * np.pi * area) / (perimetro * perimetro)
                        if circularidade > self.circularity_min:
                            shape_data.update({'label': 'Circulo', 'contour': cnt})
                            shapes_detectados.append(shape_data)
                            continue
                # Convexos -------------------------------------------------------------------------------------------------------------------------------------------------
                else:
                    # Triangulo              
                    if num_vertices == 3: 
                        if parent != -1 and child == -1: 
                            shape_data.update({'label': 'Triangulo', 'contour': approx})
                            shapes_detectados.append(shape_data)

                    # Quadrado
                    elif num_vertices == 4:  
                        angulos = [self.angulo_cos(approx[j][0], approx[(j+1)%4][0], approx[(j+2)%4][0]) for j in range(4)]
                        if all(self.quad_angle_min <= ang <= self.quad_angle_max for ang in angulos):
                            rect = cv2.minAreaRect(approx)
                            (w, h) = rect[1]
                            espectro_ratio = float(w) / h if h == 0 else (float(w) / h if w > h else float(h) / w)
                            
                            if self.min_aspect_ratio <= espectro_ratio <= self.max_aspect_ratio:
                                if parent != -1 and child == -1: # Quadrado Interno
                                    shape_data.update({'label': 'Quadrilatero', 'contour': approx})
                                    shapes_detectados.append(shape_data)  

                    elif num_vertices == 5: # Pentágono e Casa
                        angulos = [self.angulo_cos(approx[j][0], approx[(j+1)%5][0], approx[(j+2)%5][0]) for j in range(5)]
                        if all(self.pent_angle_min <= ang <= self.pent_angle_max for ang in angulos):
                            shape_data.update({'label': 'Pentagono', 'contour': approx})
                        else:
                            shape_data.update({'label': 'Casa', 'contour': approx}) 
                    shapes_detectados.append(shape_data)

            return shapes_detectados, bordas_canny, frame_clahe

    def draw(self, frame, shapes_detectados):
        for shape in shapes_detectados:
            label = shape.get('label', 'Desconhecido')
            if label == 'Desconhecido':
                continue
                
            contour = shape['contour']
            area = shape['area']
            center = shape['center']
            

            color = self.color_map.get(label, (255, 255, 255)) # Padrão: branco
            text = f"{label} - {int(area)}"
            
            x, y, _, _ = cv2.boundingRect(contour)

            
            if label == "Circulo":
                (cx_c, cy_c), raio = cv2.minEnclosingCircle(contour)
                cv2.circle(frame, (int(cx_c), int(cy_c)), int(raio), color, 3)
            else:
                # Desenho padrão para polígonos
                cv2.drawContours(frame, [contour], -1, color, 3)
            
            # Desenha texto e centro
            cv2.putText(frame, text, (x, y - 10), self.font, self.font_scale, color, self.font_thickness)
            if center:
                cv2.circle(frame, center, 5, self.center_color, -1)

        return frame


