#!/usr/bin/env python3
"""
ne pas utiliser ce prg pr l'instant. il y a trop d'incertitude
"""

print("[INFO] Importation des modules")
import asyncio
import random as rd
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw, AccelerationNed)
import time as t
import cv2
from picamera2 import Picamera2
import numpy as np
print("[INFO] Modules importés")

on_drone_reel = True
debuggage = False
want_aruco = True

print("[INFO] Allumage de la cam")
cap = Picamera2()
#cap.preview_configuration.main.size = (1920,1080)
cap.preview_configuration.main.format = "RGB888"
cap.start()
print("[INFO] Cam ouverte")



# juste pour afficher les donnees
previous_data_ned = {'down':list(), 'north':list(), 'east':list(), 'time':list(), "speed":list()}
previous_data_body = {'down':list(), 'north':list(), 'east':list(), 'time':list(), "speed":list()}
debut = t.time()

# fonction pour bouger relativement au drone selon le nord/sud/est/ouest
async def move_by(drone:System, north:float, east:float, down:float, yaw:float = 0):
    """
    ATTENTION, FONCTION A EFFET DIRECT SUR LE DRONE; NE PAS UTILISER LA FONCTION SANS REFLECHIR AUX CONSEQUENCES
    :rq: le move_by n'active pas le offboard
    """
    global position_du_drone_souhaitee
    print("yo")
    ajout = [north, east, down]
    if await drone.offboard.is_active():
        position_du_drone_souhaitee = [previous_data_ned[cle] + ajout[i] for i, cle in enumerate(previous_data_ned.keys())]
        next_point = PositionNedYaw(*position_du_drone_souhaitee, yaw)
        speed_ned = VelocityNedYaw(1, 0, 0, 0)
        await drone.offboard.set_position_velocity_ned(next_point, speed_ned)
        # await drone.offboard.set_position_ned(next_point)
    else:
        print("offboard non activated")




async def print_position(drone:System):
    global previous_data_ned, provide_random_position
    # print(drone.telemetry.position_velocity_ned())
    async for position in drone.telemetry.position_velocity_ned():
        print(position)
        previous_data_ned["north"].append(position.position.north_m)
        previous_data_ned["down"].append(-position.position.down_m)
        previous_data_ned["east"].append(position.position.east_m)
        previous_data_ned["time"].append(t.time()-debut)
        previous_data_ned["speed"].append(np.sqrt(position.velocity.north_m_s**2+position.velocity.east_m_s**2+position.velocity.down_m_s**2))




async def recentrage_sur_aruco(drone:System, vitesse_norme=1, precision_necessaire=0.1):  # precision de 10 cm (à voir pour augmenter si on ne converge pas)
    """
    permet le recentrage du drone sur l'aruco
    :rq: on utilise la vitesse pour se recentrer sur le drone; c'est pas ouf mais c'est tout ce que l'on a
    :param: vecteur_position_aruco: tuple(2) : le vecteur e_r drone -> aruco dans la base (Nord, Est) !! ATTENTION A l'orientation de la base
    :param: vitesse_norme
    :rq: ATTENTION, le DRONE NE DOIT PAS ETRE EN MODE OFFBOARD AVANT LE DEBUT DE LA FONCTION!!!!
    :rq: en sortie de la fonction, le mode offboard EST AUTOMATIQUEMENT DESACTIVE!!!!
    """
    print("recentrage fonction active")
    last_position_detected_aruco = previous_data_ned["north"][-1], previous_data_ned["east"][-1], previous_data_ned["down"][-1]
    print("here")
    identifiant, vecteur_position_aruco = await detect_aruco()
    while np.sqrt(vecteur_position_aruco[0]**2 + vecteur_position_aruco[1] **2) > precision_necessaire:
        if np.sqrt(vecteur_position_aruco[0]**2 + vecteur_position_aruco[1] **2) > 1:
            norme = np.sqrt(vecteur_position_aruco[0]**2 + vecteur_position_aruco[1] **2)
            vecteur_position_aruco = [elt / norme for elt in vecteur_position_aruco]    # on norme la vitesse pr pas que ca dépasse 1m/s
        vitesse_body = [elt * vitesse_norme for elt in vecteur_position_aruco]
        vitesse_body = VelocityBodyYawspeed(*vitesse_body, 0, 0)
        drone.offboard.set_velocity_ned(vitesse_body)
        identifiant, vecteur_position_aruco = await detect_aruco()
        if identifiant is not None:         # si il y a détection d'un aruco
            if [2] in identifiant:      # on détecte un aruco (dans notre cas, c'est 2, à voir pour changer si on change l'aruco)
                last_position_detected_aruco = previous_data_ned["north"][-1], previous_data_ned["east"][-1], previous_data_ned["down"][-1]
                print(f"Distance to aruco: {np.sqrt(vecteur_position_aruco[0]**2 + vecteur_position_aruco[1] **2)}")
        else:   # si on ne le détecte plus ..... c'est la merde!
            print("ah merde, j'ai perdu l'aruco!!!")
            where_to_find_aruco = PositionNedYaw(*last_position_detected_aruco, 0)
            drone.offboard.set_position_ned(where_to_find_aruco)
    
    print("[INFO] Arret du offboard")
    drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    drone.offboard.stop()
    drone.action.hold()
    
    print("[INFO] Lancement du largage")

    await drone.action.land()
    
    



async def run(): #Fonction principale
    drone = System()
    #Connection avec le drone
    if on_drone_reel:
        print("[INFO] Connection sur le drone réel!!!")
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        print("[INFO] Connection sur simulation")
        await drone.connect()
    print("[INFO] Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[INFO] Connected to drone!")
            break

    #Calibration de l'altitude du RTL -> hyper important
    print('[INFO] Set Return to launch altitude')
    await drone.action.set_return_to_launch_altitude(1)
    await drone.action.set_takeoff_altitude(6)


    #Importation de la mission QGroundControl
    # mission_import_data = await \
    #     drone.mission_raw.import_qgroundcontrol_mission(
    #         "mission.plan")
    # print(f"{len(mission_import_data.mission_items)} mission items imported")
    # await drone.mission_raw.upload_mission(mission_import_data.mission_items)
    # print("[INFO] Mission uploaded")

    if not debuggage:
        #Armement du drone
        print("[INFO] Arming")
        await drone.action.arm()

        # #Démarage de la mission
        # print("[INFO] Starting mission")
        # await drone.mission_raw.start_mission()

        print("[INFO] Taking off")
        await drone.action.takeoff()
        #Tache à executer
    mission_stop_aruco_task = asyncio.ensure_future(mission_stop_aruco(drone)) #voir print_mission_progress

    running_tasks = [mission_stop_aruco_task] #running_task contient les différentes tâches qui vont s'executer lors du vol

    #Vérifie si le drone a fini sa mission /!\ ensure_future
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks)) #voir observe_is_in_air
    asyncio.ensure_future(print_position(drone))


    print("[INFO] sleeping pdt 60sec. (apres c'est termination task)")
    await asyncio.sleep(60)
    await termination_task



async def detect_aruco():
    global cap

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
            #norme = (centre_x**2 + centre_y**2)**(1/2)

            #vecteur = (centre_x/norme, centre_y/norme)
            vecteur = (centre_x, centre_y)
            print("before return")
    return ids, vecteur



#Tache a executer
async def mission_stop_aruco (drone : System):
    global want_aruco
    while True:
        if want_aruco:
            ids, vecteur = await detect_aruco()
            
            if ids is not None and [2] in ids: #verifie si l'aruco repere est le numero 2
                want_aruco = False
                print('[INFO] Aruco Detected')
                print('[INFO] Mission Stop')
                await drone.mission_raw.clear_mission() #Suprime la mission en cours
                await drone.action.hold() #Demande au drone de maintenir sa position GPS
                await asyncio.sleep(5)
                print("recentrage sur aruco")
                await recentrage_sur_aruco(drone)

            #else:
                #print("no aru")


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
    try:
    # Run the asyncio loop
    	asyncio.run(run())
    except:
        cap.stop()
