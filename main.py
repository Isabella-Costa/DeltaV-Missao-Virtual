import cv2
from cameraCapture import CameraCapture         
from shapeDetection import ShapeDetector        
from baseDetection import BaseDetector

def main():
    ALVO_SHAPE = "Estrela"  #exemplo 

    try:
        camera = CameraCapture(source=0) 
    except IOError as e:
        print(e)
        return

    shape_detector = ShapeDetector(min_area=800) 

    while True:
        ret, frame = camera.get_frame()
        if not ret:
            break

        shapes_detectado = shape_detector.detect(frame)

        found_target = None
        for shape in shapes_detectado:
            if shape['label'] == ALVO_SHAPE:
                found_target = shape
                break 

        if found_target:
            # INFORMAÇÃO PARA O SETOR DE CONTROLE:
            target_center = found_target['center']
            print(f"Alvo '{ALVO_SHAPE}' localizado em {target_center}.")


        # Todos os contornos detectados
        # debug_frame = ShapeDetector.draw_detections(frame, shapes_detectado)
        #cv2.imshow("Visao do Drone", debug_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()