import cv2                         #Importamos la libreria opencv
import mediapipe as mp             #Importamos la libreria mediapipe
import numpy as np                 #Importamos la libreria numpy  
import serial                      #Importamos la libreria Serial
import time                        #Importamos la libreria de tiempo
from math import acos, degrees     #Importamos la libreria math

print("Iniciando el programa...")
ser = serial.Serial('COM5', 115200, timeout=1)  #Inicializamos el puerto serial. mandamos el puerto, la velocidad, tiempo
time.sleep(2)                                   #Tiemmpo de esppera para que iinicialice el serial

def centro_palma(lista_coordenadas):            #Función centroide de la palma
     coordenadas = np.array(lista_coordenadas)
     centro = np.mean(coordenadas, axis=0)
     centro = int(centro[0]), int(centro[1])
     return centro

mp_drawing = mp.solutions.drawing_utils           #Obetner modúlo para procesamiento de datos
mp_drawing_styles = mp.solutions.drawing_styles   #Obtener el estilo de detección
mp_hands = mp.solutions.hands                     #Obtener el módelo y detección de manos


#camara = cv2.VideoCapture(0, cv2.CAP_DSHOW)       #Leer camara  ESP32-CAM
url = 'http://192.168.137.71/640x480.jpg'            #Obtenemos el URL de la camara 
#url =  'http://192.168.137.203/photo.jpg'

camara = cv2.VideoCapture(url)       #Crear obejto VideoCapture

winName = 'IPN_CAM'
#cv2.namedWindow(winName, cv2.WINDOW_AUTOSIZE)


# Pulgar
puntos_pulgar = [1, 2, 4]                          #Asignar puntos del pulgar

# Índice, medio, anular y meñique
puntos_bajo_palm = [0, 1, 2, 5, 9, 13, 17]      #Puntos bajos de la palma
puntos_altos_palm = [8, 12, 16, 20]               #Puntos Maximos de la palma
puntos_medios_palm = [6, 10, 14, 18]               #Puntos medios de la palma

#Diccionario de Electrodomesticos; {"Clave-llave": "Valor"}
electrodomestico = {'_': '_','0':'Exit' ,'1': 'Carga', '2': 'Lampara', '3' : 'Ventilador', '4' :'Licuadora', '5' :'Plancha'}

i_uno = 0
ti_uno = 0 
tf_uno = 0                           #Inicializamos las variables de tiempo de envio de datos al serial
i_dos  = 0
ti_dos = 0
tf_dos = 0                          
tres_seg = 0


with mp_hands.Hands(
     model_complexity=1,                            #Obtenemos el modelo de manos, en donde configuramos la confidencia de muestreo, 
     max_num_hands=1,                               #como el número de manos a detectar
     min_detection_confidence=0.5,                  #
     min_tracking_confidence=0.5) as manos:
     
     while True:
          camara.open(url)                          #Antes de capturar el frame se debe de abrir la URL
          ret, cuadro = camara.read()               #Captura del frame
          #if ret:
               #camara = cv2.rotate(camara, cv2.ROTATE_90_CLOCKWISE)
               #gris = cv2.cvtColor(camara, cv2.COLOR_BGR2GRAY)
               #cv2.imshow(winName, cuadro)
               #break
          cuadro = cv2.flip(cuadro, 1)                             #Volteo horizontal de la imagen 
          altura, ancho, _ = cuadro.shape                          # Tupla que describre las dimensiones del arreglo, num de filas(altura),  
          cuadro_rgb = cv2.cvtColor(cuadro, cv2.COLOR_BGR2RGB)     # num de columnas(ancho), cantidad de colores(RGB)
          resultados = manos.process(cuadro_rgb)                   #Para procesar un fotograma de video a deteccion y nsegumiento de manos 
          conteo_dedos = "_"                                       #Inicializa el valor de el conteo de dedos
          
          grosor = [2, 2, 2, 2, 2]
          
          #inicio 
          if resultados.multi_hand_landmarks:
               coordenadas_pulgar = []
               coordenadas_palma = [] 
               coordenadas_alto = []
               coordenadas_medio = []

               for hand_landmarks in resultados.multi_hand_landmarks:
                    for index in puntos_pulgar:
                         x = int(hand_landmarks.landmark[index].x * ancho)
                         y = int(hand_landmarks.landmark[index].y * altura)
                         coordenadas_pulgar.append([x, y])
                    
                    for index in puntos_bajo_palm:
                         x = int(hand_landmarks.landmark[index].x * ancho)
                         y = int(hand_landmarks.landmark[index].y * altura)
                         coordenadas_palma.append([x, y])

                    for index in puntos_altos_palm:
                         x = int(hand_landmarks.landmark[index].x * ancho)
                         y = int(hand_landmarks.landmark[index].y * altura)
                         coordenadas_alto.append([x, y])      

                    for index in puntos_medios_palm:
                         x = int(hand_landmarks.landmark[index].x * ancho)
                         y = int(hand_landmarks.landmark[index].y * altura)
                         coordenadas_medio.append([x, y])
                    ##########################
                    # Pulgar
                    p1 = np.array(coordenadas_pulgar[0])
                    p2 = np.array(coordenadas_pulgar[1])
                    p3 = np.array(coordenadas_pulgar[2])

                    l1 = np.linalg.norm(p2 - p3)
                    l2 = np.linalg.norm(p1 - p3)
                    l3 = np.linalg.norm(p1 - p2)

                    # Calcular el ángulo
                    angulo = degrees(acos((l1**2 + l3**2 - l2**2) / (2 * l1 * l3)))
                    pulgar = np.array(False)
                    if angulo > 150:
                         pulgar = np.array(True)
                    
                    ################################
                    # Índice, medio, anular y meñique
                    nx, ny = centro_palma(coordenadas_palma)
                    cv2.circle(cuadro, (nx, ny), 3, (0, 255, 0), 2)
                    coordinates_centroid = np.array([nx, ny])
                    coordenadas_alto = np.array(coordenadas_alto)
                    coordenadas_medio = np.array(coordenadas_medio)

                    # Distancias
                    dist_centro_altos  = np.linalg.norm(coordinates_centroid - coordenadas_alto, axis=1)
                    dist_centro_medios = np.linalg.norm(coordinates_centroid - coordenadas_medio, axis=1)
                    dif = dist_centro_altos - dist_centro_medios
                    fingers = dif > 0
                    fingers = np.append(pulgar, fingers)
                    conteo_dedos = str(np.count_nonzero(fingers==True))

                    for (i, finger) in enumerate(fingers):
                         if finger == True:
                              grosor[i] = -1
               
                    mp_drawing.draw_landmarks(
                         cuadro,
                         hand_landmarks,
                         mp_hands.HAND_CONNECTIONS,
                         mp_drawing_styles.get_default_hand_landmarks_style(),
                         mp_drawing_styles.get_default_hand_connections_style())
                    
          
                    
          ################################
          # Visualización en pantalla 
         
          #cv2.rectangle(image,start_point(x,y),end_point(x,y),color,thickness)           
          cv2.rectangle(cuadro, (470, 00), (640, 140), (200, 198, 153), -1)      #RECTANGULO DE COLOR

          #cv2.putText(image,text,org(x,y),font,fontScale,color,linetype)    
          cv2.putText(cuadro, conteo_dedos,(530, 65), 1, 5, (0, 0, 0), 2)     #TEXTO CONTEO DE DEDOS

     
          aparato = electrodomestico[ conteo_dedos ]                         #Consulta el Dicionario de electrodomesticos
          cv2.putText(cuadro, aparato,(470, 115), 1, 2, (0, 0, 0), 2)            #Muestra como texto el electrodomestico seleccionado
          
          print("finger_counter: ",conteo_dedos)

          if conteo_dedos == '1':                         #Si conteo es 1
               if i_uno == 0:                          
                    ti_uno = time.time()                  #Inicializa el tiempo Inicial 
                    i_uno = 1                             #Asigna variable de que se inicializa 
                    i_dos = 0                             #Asigna cero a todas las variables de tiempo de otros dedos
               else:
                    tf_uno = time.time()                  #Inicializa el tiempo final
                    tres_seg = round( tf_uno - ti_uno, 1) #Se obtiene el total de segundos

                    if tres_seg >= 3:                     #Si el tiempo es mayor o igual a 3 segundos
                         ser.write(b'U')                  #Se envia por el puerto serial Activación del PIN12
                         print( "Tres seg", tres_seg)     #Visualizamos que sean 3 segundos o más 
                    else:                                 #Si aún no es 3 segundos 
                         tres_seg = 0                     #Inicializa el calculo del tiempo
                         tf_uno = 0                       #Inicializa el tiempo final
                         ser.write(b'E')                  #Se envia por el puerto serial la activación del PIN12
          
          
          if conteo_dedos == '2':                         #Si conteo es 2
               if i_dos == 0:                             #Mismo proceso del conteo 1
                    ti_dos = time.time()
                    i_dos = 1
                    i_uno = 0
               else:
                    tf_dos = time.time()
                    tres_seg = round( tf_dos - ti_dos, 1) #Segundos recorridos

                    if tres_seg >= 3:
                         ser.write(b'D')
                         print( "Tres seg", tres_seg)
                    else:
                         tres_seg = 0
                         tf_dos = 0
                         ser.write(b'E')

          if conteo_dedos == '0' or conteo_dedos == '_' :                        #Si el conteo de dedos es Cero (Palma cerrada)
               i_uno = 0                                 #Inicializamos todas las variables del sistema
               ti_uno = 0                                # 
               tf_uno = 0                                # 
               i_dos = 0                                 # 
               ti_dos = 0                                #
               tf_dos = 0                                #
               ser.write(b'E')                           #Se envia por el puerto serial la desactivación 
                                                         # de todos los pines
               

          cv2.imshow("Frame", cuadro)                    #Visualizaciín del Frame cámara 
          if cv2.waitKey(1) & 0xFF == 27:                #Saber si se presiono ESC, ASCII =27, Espera la entrada durante 1 milisegundo; sisi, se cierra.
               ser.write(b'E')                           #Se envia por el puerto serial la desactivación 
               break                                     # de todos los pines            
               
camara.release()                                         #Cierra el archivo de video o el dispositivo de captura.                          
cv2.destroyAllWindows()                                  #Destruye o cierra todas las ventanas en cualquier momento después de salir del script
ser.close()       