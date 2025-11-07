import cv2
import numpy as np 
from config import JANELA_CONFIG, DESENHO, COLOR_MAP

class Visualizer:
    def __init__(self, target_label):
        self.font = DESENHO.get("font", cv2.FONT_HERSHEY_SIMPLEX)
        self.font_scale = DESENHO.get("font_scale", 0.7)
        self.font_thickness = DESENHO.get("font_thickness", 2)
        self.center_color = DESENHO.get("cor_centro", (0, 0, 255))
        self.prediction_color = DESENHO.get("cor_predicao", (0, 255, 0))
        self.color_map = COLOR_MAP
        
        # Configuração das janelas
        self.main_window = f"{JANELA_CONFIG['WINDOW_NAME_NORMAL_PREFIX']} [{target_label}]"
        self.canny_window = JANELA_CONFIG["WINDOW_NAME_CANNY"]
        self.clahe_window = JANELA_CONFIG["WINDOW_NAME_CLAHE"]
        
        cv2.namedWindow(self.main_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.canny_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.clahe_window, cv2.WINDOW_NORMAL)

        # Redimensionamento
        self.resize_width = JANELA_CONFIG["LARGURA_RESIZE"]
        self.resize_height = None # Será calculado no primeiro frame

    def _calcula_altura(self, frame_height, frame_width):
        """Calcula a altura de redimensionamento mantendo a proporção."""
        if self.resize_height is None:
            self.resize_height = int(self.resize_width * (frame_height / frame_width))
            
            # Aplica o redimensionamento às janelas
            dim = (self.resize_width, self.resize_height)
            cv2.resizeWindow(self.main_window, *dim)
            cv2.resizeWindow(self.canny_window, *dim)
            cv2.resizeWindow(self.clahe_window, *dim)
        
        return (self.resize_width, self.resize_height)

    def _draw_shape(self, frame, shape, scale_x=1.0, scale_y=1.0):
        """Desenha uma única forma, o seu centro e o seu rótulo."""
        label = shape.get('label', 'Desconhecido')
        if label == 'Desconhecido':
            return
            
        contour = shape['contour']
        area = shape['area']
        center = shape['center']
        
        color = self.color_map.get(label, (255, 255, 255)) # Padrão: branco
        
        # Escala a área 
        scaled_area = area * (scale_x * scale_y)
        text = f"{label} - {int(scaled_area)}"
        
        # Escala o contorno e o centro 
        contour_scaled = (contour * np.array([scale_x, scale_y])).astype(np.int32)
        center_scaled = (int(center[0] * scale_x), int(center[1] * scale_y))

        x, y, _, _ = cv2.boundingRect(contour_scaled) # Usa o contorno escalado
        
        if label == "Circulo":
            (cx_c, cy_c), raio = cv2.minEnclosingCircle(contour)
            
            #  Escala o centro e o raio do círculo
            cx_c_scaled = int(cx_c * scale_x)
            cy_c_scaled = int(cy_c * scale_y)
            # Tira uma média da escala para o raio
            raio_scaled = int(raio * (scale_x + scale_y) / 2.0)
            
            cv2.circle(frame, (cx_c_scaled, cy_c_scaled), raio_scaled, color, 3)
        else:
            cv2.drawContours(frame, [contour_scaled], -1, color, 3)
        
        cv2.putText(frame, text, (x, y - 10), self.font, self.font_scale, color, self.font_thickness)
        if center:
            # Usa o centro escalado 
            cv2.circle(frame, center_scaled, 5, self.center_color, -1)

    # Adicionado scale_x e scale_y 
    def _draw_base(self, frame, base_data, scale_x=1.0, scale_y=1.0):
        """Desenha a 'BASE' (círculo azul) de forma especial."""
        if base_data is None:
            return
            
        center = base_data['center']
        radius = base_data['radius']
        color = self.color_map.get("BASE", (0, 0, 255))

        # Escala o centro e o raio ---
        center_scaled = (int(center[0] * scale_x), int(center[1] * scale_y))
        radius_scaled = int(radius * (scale_x + scale_y) / 2.0)
        
        cv2.circle(frame, center_scaled, radius_scaled, color, 3)
        cv2.putText(frame, "BASE", (center_scaled[0] - 30, center_scaled[1] - radius_scaled - 10), 
                    self.font, self.font_scale, color, self.font_thickness)
        # O centro real já é desenhado pela função _draw_shape("Circulo")

    # Adicionado scale_x e scale_y 
    def _draw_kalman_prediction(self, frame, prediction, target_label, scale_x=1.0, scale_y=1.0):
        """Desenha o círculo de previsão do Kalman."""
        if prediction:
            # Escala a predição 
            pred_scaled = (int(prediction[0] * scale_x), int(prediction[1] * scale_y))
            
            # O raio 10 é fixo, não precisa escalar
            cv2.circle(frame, pred_scaled, 10, self.prediction_color, 2)
            cv2.putText(frame, f"{target_label} Predito", (pred_scaled[0] + 15, pred_scaled[1]),
                        self.font, 0.5, self.prediction_color, self.font_thickness)

    def display_frame(self, frame_original, canny_img, clahe_img, all_shapes, base_data, prediction, target_label):
        """Redimensiona, desenha tudo e mostra todas as janelas."""
        
        # Pega as dimensões originais
        h_orig, w_orig = frame_original.shape[:2]
        
        # Redimensiona os frames
        dim = self._calcula_altura(h_orig, w_orig)
        w_new, h_new = dim
        
        frame = cv2.resize(frame_original, dim, interpolation=cv2.INTER_AREA)
        canny_resized = cv2.resize(canny_img, dim, interpolation=cv2.INTER_AREA)
        clahe_resized = cv2.resize(clahe_img, dim, interpolation=cv2.INTER_AREA)
        
        frame_com_desenho = frame.copy()

        # Calcula os fatores de escala ---
        scale_x = w_new / w_orig
        scale_y = h_new / h_orig

        # 1. Desenha todas as formas detetadas
        for shape in all_shapes:
            self._draw_shape(frame_com_desenho, shape, scale_x, scale_y)
            
        # 2. Desenha a "BASE" (sobrepõe o círculo azul)
        self._draw_base(frame_com_desenho, base_data, scale_x, scale_y)
        
        # 3. Desenha a previsão do Kalman
        self._draw_kalman_prediction(frame_com_desenho, prediction, target_label, scale_x, scale_y)
        
        # 4. Mostra as janelas
        cv2.imshow(self.main_window, frame_com_desenho)
        cv2.imshow(self.canny_window, canny_resized)
        cv2.imshow(self.clahe_window, clahe_resized)