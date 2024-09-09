#!/usr/bin/env python3

import asyncio

from mavsdk import System
import mavsdk.mission_raw

import cv2
from picamera2 import Picamera2
import numpy as np

print("[INFO] Ouverture de la caméra")
cap = Picamera2()
#cap.preview_configuration.main.size = (1920,1080)
cap.preview_configuration.main.format = "RGB888"
cap.start()
print("[INFO] Caméra ouverte avec succès")

#Gestion de l'output
print("[INFO] Creation du fiichier sauvegarde de la video")
frame = cap.capture_array()
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 10
height = frame.shape[0]
width = frame.shape[1]
out = cv2.VideoWriter("retour_9_aruco.mp4", int(fourcc), int(fps), (int(width), int(height)))

want_aruco = True #Variable globale qui nous dira si l'on doit chercher ou non un aruco
on_drone_reel = True        # Modifier cette variable pour passer sur simulateur!!!


async def run(): #Fonction principale
    drone = System()
    #Connection avec le drone
    if on_drone_reel:
        print("[INFO] Trying to connect to the drone")
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        print("[INFO] SUR SIMULATEUR!!! tentative de connection")
        await drone.connect()
    print("[INFO] Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[INFO] Connected to drone!")
            break

    #Calibration de l'altitude du RTL -> hyper important
    print('[INFO] Set Return to launch altitude')
    await drone.action.set_return_to_launch_altitude(1)


    #Tache à executer
    mission_stop_aruco_task = asyncio.ensure_future(
        mission_stop_aruco(drone)) #voir print_mission_progress

    running_tasks = [mission_stop_aruco_task] #running_task contient les différentes tâches qui vont s'executer lors du vol

    #Vérifie si le drone a fini sa mission /!\ ensure_future
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks)) #voir observe_is_in_air

    #Importation de la mission QGroundControl
    mission_import_data = await \
        drone.mission_raw.import_qgroundcontrol_mission(
            "mission1.plan")
    print(f"{len(mission_import_data.mission_items)} mission items imported")
    await drone.mission_raw.upload_mission(mission_import_data.mission_items)
    print("[INFO] Mission uploaded")

    #Armement du drone
    print("[INFO] Arming")
    await drone.action.arm()

    #Démarage de la mission
    print("[INFO] Starting mission")
    await drone.mission_raw.start_mission()

    await termination_task


#Tache a executer
async def mission_stop_aruco (drone : System):
    global want_aruco
    async for position in drone.telemetry.position():
        if want_aruco:
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
            if ids is not None and ([2] in ids or [1] in ids or [3] in ids or [4] in ids): #verifie si l'aruco repere est le numero 2
                image_with_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
                want_aruco = False
                print('[INFO] Aruco Detected')
                print('[INFO] Mission Stop')
                await drone.mission_raw.clear_mission() #Suprime la mission en cours
                await drone.action.hold() #Demande au drone de maintenir sa position GPS
                await asyncio.sleep(10)
                print('[INFO] Return to Launch')
                await drone.action.return_to_launch()
                # Sauvegarde
                out.write(image_with_markers)
            else:
                # Sauvegarde
                out.write(frame)
                print("no aru")


async def observe_is_in_air(drone, running_tasks): #Met fin au programme si le drone est au sol
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False #Avant de décollet, le drone est au sol

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air: # = Si le drone vient d'attérir
            print('[INFO] Drone on Ground')
            for task in running_tasks:
                task.cancel() #Supprime toutes les tâches en cours d'execution
                try: #vérifie si la tâche à bien été supprimée
                    await task 
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens() #Met fin à la boucle asyncio et donc fin au programme
            cap.stop() #Arret de la caméra

            return


if __name__ == "__main__":
    # Run the asyncio loop
    try:
        asyncio.run(run())
    except:
        cap.stop()
