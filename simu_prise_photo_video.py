# importation des modules
print("[INFO] Importation des modules")
import asyncio
import random as rd
import time as t
import cv2
from picamera2 import Picamera2
import numpy as np
import json
print("[INFO] Modules importés")


# allumage de la caméra
print("[INFO] Allumage de la cam")
cap = Picamera2()
cap.preview_configuration.main.format = "RGB888"
cap.start()
print("[INFO] Cam ouverte")

# init des variables globales
frame = None
ids, vecteur = None, None

# gestion du flux video pour enregistrement des images (sous forme video)
print("[INFO] Creation du fiichier sauvegarde de la video")
path_to_video = "video_cam.mp4"
frame = cap.capture_array()
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 30
height = frame.shape[0]
width = frame.shape[1]
out = cv2.VideoWriter(path_to_video, int(fourcc), int(fps), (int(width), int(height)))

async def recentrage_sur_aruco():
    prev_frame = frame[0][0]
    for i in range(300):
        print(prev_frame, frame[0][0])
        if np.all(frame[0][0] != prev_frame):
            print(ids, vecteur)
        prev_frame = frame[0][0]
        await asyncio.sleep(0.1)


async def detect_aruco_on_image(frame):
    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
    # Définir le dictionnaire ArUco
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    # Paramètres de détection des marqueurs ArUco
    parameters =  cv2.aruco.DetectorParameters_create()
    # Détecter les marqueurs ArUco

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    vecteur = None

    if ids is not None:
        print(f"[INFo] Aruco detected {ids}")
        # Recherche le centre de l'aruco
        if [2] in ids:
            num = np.where(ids == [2])[0][0]
            data = corners[num][0]
            list_x = []
            list_y = []
            for coin in data:
                list_x.append(coin[0])
                list_y.append(coin[1])
            centre_x = int((max(list_x) + min(list_x))/2) - frame.shape[1]//2
            centre_y = int((max(list_y) + min(list_y))/2) - frame.shape[0]//2
            vecteur = (centre_x, centre_y)
    return ids, vecteur, corners

async def prise_video_cam(extract_aruco=True):
    """
    fonction mise en ensure_future qui gère la prise de photo/vidéo
    :param: capture_video: if True: l'ensemble des photos prises sont enregistrées et sont sauvegardées sous un format de vidéo
    """
    global cap, out, frame, ids, vecteur
    while True:
        await asyncio.sleep(1/fps)
        # Lire une frame de la vidéo
        frame = cap.capture_array()
        if extract_aruco:
            ids, vecteur, corners = await detect_aruco_on_image(frame)
            frame = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        if path_to_video:
            out.write(frame)    # TODO: faire l'enregistrement du flux vidéo
        if extract_aruco:
            ids, vecteur = ids, vecteur



async def run(): #Fonction principale
    global out

    print("[INFO] Activation de la vidéo!!!")
    recording_video_fut_task = asyncio.ensure_future(prise_video_cam(extract_aruco=True))
    await asyncio.sleep(5)

    print("[INFO] simu recentrage")
    await recentrage_sur_aruco()
                
    # gestion de la fermeture de la vidéo
    print("[INFO] Fermeture de la vidéo")
    recording_video_fut_task.cancel()
    out.release()
    print("[INFO] Programme terminé!!!")
    return
    
    
if __name__ == "__main__":
    try:
        asyncio.run(run())
    except Exception as error:
        print(error)
        cap.close()
