#!/usr/bin/env python3

"""
programme de test du recentrage sur aruco
le principe du programme est de décollé à 4m, et de chercher à détecter un code aruco
dès qu'il y en a un de détecter on s'y positionne
puis on atterit dessus
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw, AccelerationNed)
import matplotlib.pyplot as plt
import numpy as np
import time as t
import random as rd
import cv2
from picamera2 import Picamera2
import numpy as np

un_aruco_a_ete_detecte = False

print("[INFO] Ouverture de la caméra")
cap = Picamera2()
#cap.preview_configuration.main.size = (1920,1080)
cap.preview_configuration.main.format = "RGB888"
cap.start()
print("[INFO] Caméra ouverte")

want_aruco = True #Variable globale qui nous dira si l'on doit chercher ou non un aruco
debuggage = True    # si True, on se connecte au drone mais on n'active pas les moteurs.


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
    if await drone.offboard.is_active() or debuggage:
        position_du_drone_souhaitee = [previous_data_ned[cle] + ajout[i] for i, cle in enumerate(previous_data_ned.keys())]
        next_point = PositionNedYaw(*position_du_drone_souhaitee, yaw)
        speed_ned = VelocityNedYaw(1, 0, 0, 0)
        if not debuggage : await drone.offboard.set_position_velocity_ned(next_point, speed_ned)
        # await drone.offboard.set_position_ned(next_point)
    else:
        print("offboard non activated")




async def print_position(drone:System):
    global previous_data_ned, provide_random_position
    # print(drone.telemetry.position_velocity_ned())
    async for position in drone.telemetry.position_velocity_ned():
#        print(position)
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
    identifiant, vect_temp = await detect_aruco()
    if identifiant is not None and [2] in identifiant:
        vecteur_position_aruco = vect_temp
    if identifiant is not None and not debuggage:
        if [2] in identifiant:
            last_position_detected_aruco = previous_data_ned["north"][-1], previous_data_ned["east"][-1], previous_data_ned["down"][-1]
    while np.sqrt(vecteur_position_aruco[0]**2 + vecteur_position_aruco[1] **2) > precision_necessaire:
        vitesse_body = [elt * vitesse_norme for elt in vecteur_position_aruco]
        vitesse_body = VelocityBodyYawspeed(*vitesse_body, 0, 0)
        if not debuggage: drone.offboard.set_velocity_body(vitesse_body)
        identifiant, vecteur_position_aruco = await detect_aruco()
        if identifiant is not None:         # si il y a détection d'un aruco
            if [2] in identifiant:      # on détecte un aruco (dans notre cas, c'est 2, à voir pour changer si on change l'aruco)
                if not debuggage: last_position_detected_aruco = previous_data_ned["north"][-1], previous_data_ned["east"][-1], previous_data_ned["down"][-1]
                print(f"Distance to aruco: {np.sqrt(vecteur_position_aruco[0]**2 + vecteur_position_aruco[1] **2)}, vitesse_boyd: {vitesse_body}")
        else:   # si on ne le détecte plus ..... c'est la merde!
            if not debuggage:
                print("ah merde, j'ai perdu l'aruco!!!")
                where_to_find_aruco = PositionNedYaw(*last_position_detected_aruco, 0)
                if not debuggage: drone.offboard.set_position_ned(where_to_find_aruco)
                await asyncio.sleep(1)
    if not debuggage:
        drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        drone.offboard.stop()
        drone.action.hold()
    return
    

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
    print("yo", ids)
    if ids is not None:
        # Recherche le centre de l'aruco
        print("hello", ids)
        if [2] in ids:
            num = np.where(ids == [2])[0][0]
            data = corners[num][0]
            list_x = []
            list_y = []
            for coin in data:
                list_x.append(coin[0])
                list_y.append(coin[1])
            centre_x = int((max(list_x) + min(list_x))/2) -frame.shape[1]
            centre_y = int((max(list_y) + min(list_y))/2) -frame.shape[0]
            norme = (centre_x**2 + centre_y**2)**(1/2)

            vecteur = (centre_x/norme, centre_y/norme)

    return ids, vecteur


async def stop_mission_and_recentrate_on_aruco(drone:System):
    global un_aruco_a_ete_detecte
    print("je suis lance")
    ids, vecteur = await detect_aruco()
    if ids is not None and [2] in ids:
        if not un_aruco_a_ete_detecte: print("Aruco détecté; arrêt de la mission et recentrage sur l'aruco")
        un_aruco_a_ete_detecte = True


async def run():
    print("connecting")
    
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyAMA0:57600")      # mettre la bonne adresse
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    if not debuggage:
        print("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break
    asyncio.ensure_future(print_position(drone))
    await drone.action.set_takeoff_altitude(4)
    
    if not debuggage: 
        print("-- Arming")
        await drone.action.arm()

        await drone.action.takeoff()
        print("taking off")
        await asyncio.sleep(8)
        print("sleeping")
        await drone.action.hold()
    print("hold et attente d'aruco")

    while not un_aruco_a_ete_detecte and (t.time() - debut < 60): # si il n'y a pas eu d'aruco détecté ET que le drone a attendu moins de 60s
        print("aucun aruco détecté")
        await asyncio.sleep(0.1)
        await stop_mission_and_recentrate_on_aruco(drone)
    
    if un_aruco_a_ete_detecte :
        print("un aruco a été détecté; recentrage activé")
        await recentrage_sur_aruco(drone)

    if not debuggage: await drone.action.land()




if __name__ == "__main__":
    asyncio.run(run())
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    scatter = ax.scatter(previous_data_ned["north"], previous_data_ned["east"], previous_data_ned["down"], c=previous_data_ned["speed"], cmap = 'viridis')
    cbar = fig.colorbar(scatter, ax=ax, label='Color based on value')
    plt.legend()
    plt.show()
