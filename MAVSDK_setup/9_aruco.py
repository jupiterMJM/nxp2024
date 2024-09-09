import cv2
from picamera2 import Picamera2
import numpy as np

cap = Picamera2()
#cap.preview_configuration.main.size = (1920,1080)
cap.preview_configuration.main.format = "RGB888"
cap.video_configuration.controls.FrameRate = 30.0
cap.start()

#Gestion de l'output
frame = cap.capture_array()
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 10
height = frame.shape[0]
width = frame.shape[1]
out = cv2.VideoWriter("retour_9_aruco.mp4", int(fourcc), int(fps), (int(width), int(height)))

# Boucle de capture vidéo
while True:
    # Lire une frame de la vidéo
    frame = cap.capture_array()
    

    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Définir le dictionnaire ArUco
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    # Paramètres de détection des marqueurs ArUco
    parameters =  cv2.aruco.DetectorParameters_create()
    # Détecter les marqueurs ArUco
 
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    # Dessiner les marqueurs détectés
    if ids is not None and [2] in ids: ### Normalement ids est un tableau d'entier, chaque entier représentant un aruco détecté
        image_with_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        print("[INFO] Aruco Detected :", ids)
        # Sauvegarde
        out.write(image_with_markers)
    else : 
        # Sauvegarde
        out.write(frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer la capture et fermer les fenêtres
cap.stop()
cv2.destroyAllWindows()
