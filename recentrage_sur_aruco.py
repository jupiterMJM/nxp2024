#!/usr/bin/env python3
"""
ne pas utiliser ce prg pr l'instant. il y a trop d'incertitudes
:TODO: séparer ce programme 
ATTENTION: https://stackoverflow.com/questions/74964527/attributeerror-module-cv2-aruco-has-no-attribute-dictionary-get
    module python mis à jour, l'API a été changée
"""

# importation des modules
print("[INFO] Importation des modules")
import asyncio
import random as rd
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw, AccelerationNed)
import time as t
import cv2
from picamera2 import Picamera2
import numpy as np
import json
print("[INFO] Modules importés")

# variables globales à modif pour gestion du programme
try_connection_with_drone = False       # indique s'il faut se connecter au drone; mettre à False quand il faut juste faire des tests de caméra
on_drone_reel = True                    # indique si on se connecte au simu ou au drone
debuggage = False                       # activer le mode debuggage pour faire des tests sans que le drone ne décolle (en gros, ca active juste la caméra et l'algo)


# au cas où....
debuggage = debuggage and try_connection_with_drone

# initialisation des variables globales
current_info = list()  # north_m, east_m, down_m, speed
historic = {"north":list(), "east":list(), "down":list(), "speed":list()}
last_time_qr_code_seen = [None, None, None]

# allumage de la caméra
print("[INFO] Allumage de la cam")
cap = Picamera2()
cap.preview_configuration.main.format = "RGB888"
cap.start()
print("[INFO] Cam ouverte")


# gestion du flux vidéo pour enregistrement des images (sous forme vidéo)
print("[INFO] Creation du fiichier sauvegarde de la video")
path_to_video = "video_cam.mp4"
frame = cap.capture_array()
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 30
height = frame.shape[0]
width = frame.shape[1]
out = cv2.VideoWriter(path_to_video, int(fourcc), int(fps), (int(width), int(height)))

# initialisation des variables globales nécessaires au programme
frame = cap.capture_array()
ids, vecteur = None, None


async def goto_in_ned_absolute(drone:System, drone_position_aim, velocity, tolerance=0.2, little_sleep=0.1):
    pattern_velocity = lambda x: velocity if x > 1 else 0.5
    # on recupere les coordonnees initiales du drone
    get_position = lambda : current_info[:-1]
    get_speed_norm = lambda : current_info[-1]

    drone_position_init = get_position()
    # on enregistre cette info même si save_traj==False (dans ce cas là, c'est juste que l'info ne sera pas enregistré)
    historic["Info"] = [*drone_position_aim, velocity]
    # puis on boucle jusqu'à ce qu'on y soit
    while True:
        drone_position_current = np.array(get_position())
        vecteur_dir = drone_position_aim - drone_position_current
        distance_to_aim = np.linalg.norm(vecteur_dir)
        print(f"[INFO] Distance to aim: {distance_to_aim}")
        if distance_to_aim < tolerance:
            await drone.offboard.set_position_ned(PositionNedYaw(drone_position_aim[0], drone_position_aim[1], drone_position_aim[2], 0.0))
            print("[INFO] Should be arrived")
            await asyncio.sleep(3)
            break
        vecteur_unit = vecteur_dir / np.linalg.norm(vecteur_dir)

        # puis on dirige le groupe
        next_position = drone_position_current + vecteur_unit * pattern_velocity(distance_to_aim)
        await drone.offboard.set_position_ned(PositionNedYaw(next_position[0], next_position[1], next_position[2], 0.0))
        await asyncio.sleep(little_sleep)



# fonction pour bouger relativement au drone selon le nord/sud/est/ouest
async def move_in_ned_with_velocity(drone:System, aiming_pos, velocity, tolerance=0.2, little_sleep = 0.1):
    """
    aiming_pos: la position RELATIVE au drone actuellement sous (pos_sn, pos_we, pos_hb)
    cad que pour (x, y, z) le drone ira à x mètre vers le QUOI, y metre vers le QUOI, z metre vers le QUOI
    le drone bougera avec une vitesse de velocity
    :comment: fonction issue du fichier *move_in_ned_with_velocity* il est recommandé de bien faire attention à ce que cette fonction soit à la version la plus récente
    """
    
    pattern_velocity = lambda x: velocity if x > 1 else 0.5
    # on recupere les coordonnees initiales du drone
    get_position = lambda : current_info[:-1]
    get_speed_norm = lambda : current_info[-1]

    drone_position_init = get_position()
    drone_position_aim = np.array(drone_position_init) + np.array(aiming_pos)
    # on enregistre cette info même si save_traj==False (dans ce cas là, c'est juste que l'info ne sera pas enregistré)
    historic["Info"] = [*drone_position_aim, velocity]
    # puis on boucle jusqu'à ce qu'on y soit
    while True:
        drone_position_current = np.array(get_position())
        
        speed = get_speed_norm()
        vecteur_dir = drone_position_aim - drone_position_current
        distance_to_aim = np.linalg.norm(vecteur_dir)
        print(f"[INFO] Distance to aim: {distance_to_aim}")
        if distance_to_aim < tolerance:
            await drone.offboard.set_position_ned(PositionNedYaw(drone_position_aim[0], drone_position_aim[1], drone_position_aim[2], 0.0))
            print("[INFO] Should be arrived")
            await asyncio.sleep(3)
            break
        vecteur_unit = vecteur_dir / np.linalg.norm(vecteur_dir)

        # puis on dirige le groupe
        next_position = drone_position_current + vecteur_unit * pattern_velocity(distance_to_aim)
        await drone.offboard.set_position_ned(PositionNedYaw(next_position[0], next_position[1], next_position[2], 0.0))
        await asyncio.sleep(little_sleep)



async def move_in_frd_with_velocity(drone:System, aiming_pos, velocity, tolerance=0.2, little_sleep=0.1):
    """
    fonction permettant de controler le drone dans sa base locale (Forward, Right, Down)
    fonction testée et approuvée sous simulateur
    :comment: on change les coord de base
        de FRD à NED
    """
    # obtention de l'angle phi entre l'axe nord et l'axe forward
    async for heading_data in drone.telemetry.heading():
        phi = np.deg2rad(heading_data.heading_deg)
        break   # on a besoin de la donnée qu'une seule fois

    # chgt de base
    along_north_axis = aiming_pos[0] * np.cos(phi) - aiming_pos[1] * np.sin(phi)
    along_east_axis = aiming_pos[0] * np.sin(phi) + aiming_pos[1] * np.cos(phi)

    # envoi de la commande à la fonction move_in_ned_with_velocity
    await move_in_ned_with_velocity(drone, (along_north_axis, along_east_axis, aiming_pos[2]), velocity, tolerance, little_sleep)
  


# fonction en ensure_future pour gérer où est le drone
async def save_trajectory_in_ned(drone, save_traj=True, keep_one_on=1, backup=100):
    """
    :param: le backup est effectué tous les *backup* iterations
    :param: on garde une donnée sur *keep_one_on*
    """
    global historic, current_info
    indic = 0
    async for position_ned in drone.telemetry.position_velocity_ned():
        indic += 1
        if indic < 10:
            print(f"[TEST URGENT] {indic}e  donnée prise")
        current_info = position_ned.position.north_m, position_ned.position.east_m, position_ned.position.down_m,np.linalg.norm(np.array([position_ned.velocity.north_m_s, position_ned.velocity.east_m_s, position_ned.velocity.down_m_s]))

        if save_traj and indic % keep_one_on == 0:
            historic["north"].append(current_info[0])
            historic["east"].append(current_info[1])
            historic["down"].append(current_info[2])
            
            historic["speed"].append(current_info[3])
        if save_traj and indic % backup == 0:
            with open(f"trajectory.json", "w") as f:
                json.dump(historic, f)
                print("[INFO] Trajectory saved (backup)")


async def recentrage_sur_aruco(drone:System, follow_aruco=False):  # precision de 10 cm (à voir pour augmenter si on ne converge pas)
    """
    permet le recentrage du drone sur l'aruco
    :rq: on utilise la vitesse pour se recentrer sur le drone; c'est pas ouf mais c'est tout ce que l'on a
    :param: vecteur_position_aruco: tuple(2) : le vecteur e_r drone -> aruco dans la base (Nord, Est) !! ATTENTION A l'orientation de la base
    :param: follow_aruco : if True: on ne sort "jamais" de la boucle, ce qui fait que l'on suit le qrcode
    :rq: ATTENTION, le DRONE NE DOIT PAS ETRE EN MODE OFFBOARD AVANT LE DEBUT DE LA FONCTION!!!!
    :rq: en sortie de la fonction, le mode offboard EST AUTOMATIQUEMENT DESACTIVE!!!!
    """
    await asyncio.sleep(1)
    print("[INFO] Recentrage sur aruco activé!!!!!")
    prev_frame = frame[0][0]
    while True:
        if np.all(prev_frame != frame[0][0]):       # cad si l'image a été update
            prev_frame = frame[0][0]
            if [2] in ids:
                last_time_qr_code_seen = current_info[:-1]          # cf move_in_ned function
            else:
                if not None in last_time_qr_code_seen:
                    print("[INFO] Aruco perdu; retour à la dernière position connue")
                    await goto_in_ned_absolute(drone, last_time_qr_code_seen)
                    await asyncio.sleep(5)
                continue

            vecteur_norme = np.array(vecteur)/np.linalg.norm(vecteur)
            vecteur_consigne = 0.10 * vecteur_norme             # en gros on bouge de 10cm en 10cm
            # on fait bouger le drone
            if not debuggage: move_in_frd_with_velocity(drone, (vecteur_consigne[0], vecteur_consigne[1], 0), 0.5)

            if not follow_aruco and np.linalg.norm(vecteur) < 0.1: # pas très bien mais à voir
                # on considère qu'on est au dessus du tag
                return


async def run(): #Fonction principale
    global out

    drone = System()
    #Connection avec le drone
    if try_connection_with_drone:
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

        if not debuggage:
            #Armement du drone
            print("[INFO] Arming")
            await drone.action.arm()

        print("[INFO] Activation de la récolte des données NED")
        saving_traj_ensure_fut_task = asyncio.ensure_future(save_trajectory_in_ned(drone, save_traj=True))
    print("[INFO] Activation de la vidéo!!!")
    recording_video_fut_task = asyncio.ensure_future(prise_video_cam(extract_aruco=True))
    await asyncio.sleep(2)

    if not debuggage and try_connection_with_drone:
        print("[INFO] Taking off")
        await drone.action.takeoff()

        #Tache à executer
        await recentrage_sur_aruco(drone)           # WARNING, pas très sur de ce qu'il se passe à ce moment là!!!

        print("[INFO] RTL activé")
        # il faut éteindre après l'atterissage
        await drone.action.return_to_launch()
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                await asyncio.sleep(1)
                # gestion du .json de la sauvegarde de traj
                saving_traj_ensure_fut_task.cancel()
                with open(f"trajectory.json", "w") as f:
                    json.dump(historic, f)
                    print("[INFO] Trajectory saved (backup)")

                print("[INFO] Disarming")
                await drone.action.disarm()
                
    # gestion de la fermeture de la vidéo
    print("[INFO] Fermeture de la vidéo")
    recording_video_fut_task.cancel()
    out.release()
    print("[INFO] Programme terminé!!!")
    return
    


    
async def prise_video_cam(extract_aruco=True):
    """
    fonction mise en ensure_future qui gère la prise de photo/vidéo
    :param: capture_video: if True: l'ensemble des photos prises sont enregistrées et sont sauvegardées sous un format de vidéo
    fonctnio 
    """
    global cap, out, frame, ids, vecteur
    while True:
        await asyncio.sleep(1/fps)
        # Lire une frame de la vidéo
        frame = cap.capture_array()
        if extract_aruco:
            ids, vecteur, corners = detect_aruco_on_image(frame)
            frame = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        if path_to_video:
            out.write(frame)



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






if __name__ == "__main__":
    try:
        # Run the asyncio loop
        asyncio.run(run())
    except Exception as error:
        print(error)
        out.release()
        cap.stop()
