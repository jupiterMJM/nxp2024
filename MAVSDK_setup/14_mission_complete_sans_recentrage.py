#!/usr/bin/env python3

import asyncio

from mavsdk import System
import mavsdk.mission_raw
import serial
import cv2
from picamera2 import Picamera2
import numpy as np
import time
from collections import deque

print("[INFO] Ouverture de la camera")
cap = Picamera2()
#cap.preview_configuration.main.size = (1920,1080)
cap.preview_configuration.main.format = "RGB888"
cap.start()
print("[INFO] Camera ouverte avec succes")

#Gestion de l'output
print("[INFO] Creation du retour video")
frame = cap.capture_array()
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 10
height = frame.shape[0]
width = frame.shape[1]
out = cv2.VideoWriter("retour_video.mp4", int(fourcc), int(fps), (int(width), int(height)))

want_aruco = True #Variable globale qui nous dira si l'on doit chercher ou non un aruco
on_drone_reel = True        # Modifier cette variable pour passer sur simulateur!!!
startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False
debuggage = True
# sauvegarde du lidar
hauteur_lidar = deque(maxlen=10)


#========================
#========================
    # the functions

def setupSerial(baudRate, serialPortName):
    
    global  serialPort
    
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)

    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))

   #waitForArduino()

#========================

def sendToArduino(stringToSend):
    
        # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort
    
    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3


#==================

def recvLikeArduino():

    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") # decode needed for Python3
        
        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True
    
    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX" 

#==================

def waitForArduino():

    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    
    print("Waiting for Arduino to reset")
     
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'): 
            print(msg)



#====================
#====================
    # the program

# fonction de maj des donnees du drone
async def update_lidar(drone):
    global hauteur_lidar
    async for distance in drone.telemetry.distance_sensor():
        for i in range(2):
            hauteur_lidar.append(distance.current_distance_m)

async def lancement_treuillage():
    setupSerial(115200, "/dev/ttyAMA3")
    prevTime = time.time()
    mission_lancee = False
    try:
        while True:
                    # check for a reply
            arduinoReply = recvLikeArduino()
            if not (arduinoReply == 'XXX'):
                print ("Time %s  Reply %s" %(time.time(), arduinoReply))
                if arduinoReply ==  "statut mission finie":
                    break
                # send a message at intervals

            messages_arduino = arduinoReply.split(" ")
            if len(messages_arduino) > 1:
                if messages_arduino[1] == "mission":
                    mission_lancee = True
                    
                    
            if (time.time() - prevTime > 1.0) and (mission_lancee == False):
                print("j'essaie d'envoyer des choses a l'arduino")
                max_hauteur_lidar = max(hauteur_lidar) - 0.15 - 0.25
                max_hauteur_lidar = str(int(max_hauteur_lidar*1000))
                msg = f"cmd mission_treuil {max_hauteur_lidar} false"
                print(f"cmd qui est envoyee: {msg}")
                sendToArduino(f"cmd mission_treuil {max_hauteur_lidar} false")
                prevTime = time.time()

        print("fin de communication")
    except:
        while True:
            sendToArduino("sec abort")
            
    finally:
        print("fin de mission")
        


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
    asyncio.ensure_future(update_lidar(drone))
    if not debuggage:
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"[INFO] Connected to drone!")
                break

        #Calibration de l'altitude du RTL -> hyper important
        print('[INFO] Set Return to launch altitude')
        await drone.action.set_return_to_launch_altitude(1)

    if not debuggage:

        print("[INFO] en train d'upload les missions")
        #Verifie si le drone a fini sa mission /!\ ensure_future
        print('[INFO] Importing Mission')
        mission_import_data = await drone.mission_raw.import_qgroundcontrol_mission("mission1.plan")
        print("Salut")
        print(f"{len(mission_import_data.mission_items)} mission items imported")
        await drone.mission_raw.upload_mission(mission_import_data.mission_items)
        print("[INFO] Mission uploaded")
            
    # tache a executer APRES l'init du drone
    if not debuggage:
        #Armement du drone
        print("[INFO] Arming")
        await drone.action.arm()

    print("[INFO] Mise en ensure_future de mission_stop_aruco")
    mission_stop_aruco_task = asyncio.ensure_future(mission_stop_aruco(drone)) #voir print_mission_progress
    running_tasks = [mission_stop_aruco_task] #running_task contient les differentes tâches qui vont s'executer lors du vol

    if not debuggage:
        #Demarage de la mission
        print("[INFO] Starting mission")
        
        await drone.mission_raw.start_mission()
        termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks)) #voir observe_is_in_air
        await termination_task
        
    else: # pour pouvoir gerer le debuggage
        await asyncio.sleep(60)


#Tache a executer
async def mission_stop_aruco (drone : System):
    global want_aruco
    #async for position in drone.telemetry.position():
    while True:
        if want_aruco:
            # Lire une frame de la video
            frame = cap.capture_array()

            # Convertir l'image en niveaux de gris
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    
            # Definir le dictionnaire ArUco
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            # Parametres de detection des marqueurs ArUco
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detecter les marqueurs ArUco

            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

            # Dessiner les marqueurs detectes
            if ids is not None and ([2] in ids or [1] in ids or [3] in ids or [4] in ids): #verifie si l'aruco repere est le numero 2
                want_aruco = False
                image_with_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
                out.write(image_with_markers) # Sauvegarde de l'image
                print('[INFO] Aruco Detected')
                if not debuggage:
                    print('[INFO] Mission Stop')
                    await drone.mission_raw.clear_mission() # Suprime la mission en cours
                    print('[INFO]Mission retiree')
                    await drone.action.hold() # Demande au drone de maintenir sa position GPSprint("[INFO] hold et attente un petit peu")
                await asyncio.sleep(5)
                print("[INFO] Lancement du treuillage")
                await lancement_treuillage() 
                print("[INFO] Treuillage termine")
                await asyncio.sleep(5)
                print("[INFO] Fin du programme")
                if not debuggage:
                    print('[INFO] Return to Launch')
                    await drone.action.return_to_launch()
#                else:
#                    en_cours = True
#                    while en_cours:
#                        try:
#                            await asyncio.get_event_loop().shutdown_asyncgens() #Met fin a la boucle asyncio et donc fin au programme
#                        except:
#                            print("tentative de deco")
            else:
                #print("no aru")
                # Sauvegarde de l'image
                out.write(frame)


async def observe_is_in_air(drone, running_tasks): #Met fin au programme si le drone est au sol
    """ Met fin aux taches du drone lors de l'atterisssage """

    was_in_air = False #Avant de decollet, le drone est au sol

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air: # = Si le drone vient d'atterir
            print('[INFO] Drone au sol')
            for task in running_tasks:
                task.cancel() #Supprime toutes les tâches en cours d'execution
                try: #verifie si la tâche a bien ete supprimee
                    await task 
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens() #Met fin a la boucle asyncio et donc fin au programme
            cap.stop() #Arret de la camera

            return


if __name__ == "__main__":
    # Run the asyncio loop
    try:
        asyncio.run(run())
    except:
        cap.stop()
