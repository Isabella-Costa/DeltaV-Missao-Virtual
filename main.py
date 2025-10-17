import cv2

from cameraCapture import CameraCapture
from shapeDetection import ShapeDetection
from baseDetection import BaseDetector 

def main():
    ALVO_SHAPE = "Estrela" 

    try:
        camera = CameraCapture(source=0)
    except IOError as e:
        print(e)
        return

    shape_detector = ShapeDetection(min_area=800)
    base_detector = BaseDetector() 

    mission_status = 'Alinhado com a base'
    delivery_start_time = 0
    
    print(f"COMEÇOU A missão.  Alvo: {ALVO_SHAPE}. Pressione 'q' para sair.")

    while True:
        ret, frame = camera.get_frame()
        if not ret:
            break

        # Centro da visão do drone 
        center_x = camera.width // 2
        center_y = camera.height // 2
        

        # ESTADO 1: Procurando a figura-alvo
        if mission_status == 'Procura Alvo':
            pass
        
        # ESTADO 2: Alinhando sobre o alvo para entregar a carga
        elif mission_status == 'Alinha com Alvo':
            pass
        # ESTADO 3: Simulando a entrega da carga
        elif mission_status == 'Entrega no Alvo':
            pass
        # ESTADO 4: Procurando a base de pouso
        elif mission_status == 'Retornando a base':
            pass
        # ESTADO 5: Alinhando sobre a base para pousar
        elif mission_status == 'Alinhado com a base':
            pass
        # ESTADO 6: Missão Concluída
        elif mission_status == 'Fim da missão':
            pass

        # para exibir o status da missão 
        cv2.putText(frame, f"STATUS: {mission_status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("Visao do Drone", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()
    print("Missão encerrada pelo operador.")

if __name__ == "__main__":
    main()