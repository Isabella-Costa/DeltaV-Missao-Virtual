import cv2
import numpy as np
import deteccao_forma as detc

def main():

    captura = cv2.VideoCapture(0)
        
    validacao, frame = captura.read() 

    while validacao: 
        validacao, frame = captura.read()

        if not validacao:
            print("Erro: Nao foi possivel ler o frame da câmera.")
            break

        frame_copia = frame.copy()

        quadrados_internos, triangulos_internos, bordas_canny, img_clahe = detc.detectar_formas(frame_copia)

        detc.desenhar(frame_copia, quadrados_internos)
        detc.desenhar_triangulos(frame_copia, triangulos_internos)
        
        #cv2.imshow("thresholding adaptativo", frame_TA)
        cv2.imshow("canny", bordas_canny)
        cv2.imshow("resultado final", frame_copia)
        cv2.imshow("CLAHE", img_clahe)

        
        key = cv2.waitKey(5) # faz o frame esperar x milissegundos e armazena a tecla 
        if key == 27: #ESC
            break 

    captura.release() # finaliza a conexão com a webcam 
    cv2.destroyAllWindows() # fechar a janela 

if __name__ == "__main__":
    main()

    #propriedades da cam
    #fov da cam