"""
programme de test;
il s'agit de vérifier que angle=0 <=> le drone pointe plein nord
"""

#Importation des bibliothèques
import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

drone_reel_horus = False

async def run():
    '''
    Fonction principales : définies à l'aide de async (fonction asynchrone)
    C'est elle qui va gérer la création de la mission et son exécution
    '''

    #Connection au drone
    drone = System() #Le drone est un objet de la class "System" de MAVSDK
    print("[INFO] Connecting to the drone")
    if drone_reel_horus:
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        await drone.connect()
        print("[WARNING] lancement sur simulateur")
    #Attente de la connection
    print("[INFO] Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[INFO] Connected to drone!")
            break
    
    # #Programation de l'altitude voulue pour le décollage
    # drone.action.set_takeoff_altitude(1.5)
    
    # print("[INFO] Arming")
    # await drone.action.arm()
    
    # #Décollage du drone
    # print("[INFO] Taking off")
    # await drone.action.takeoff()

    # await asyncio.sleep(5)
    #Test de hold dans le take_off, partie a cammenter si besoin
    ##################################################################
    async for heading_data in drone.telemetry.heading():
        heading_angle = heading_data.heading_deg  # Get the heading in degrees
        print(f"Current heading (angle relative to north): {heading_angle} degrees")
    ##################################################################
    
    # await asyncio.sleep(10) #temps en seconde avant de demander au drone de se poser
    
    # #Atterissage
    # print("[INFO] Landing")
    # await drone.action.land()
    
    # await asyncio.get_event_loop().shutdown_asyncgens()



if __name__ == "__main__":
    # Run the asyncio loop
    print("[INFO] Programme Takeoff et Land")
    confirm_execute = input("Confirmer lancement programme (y/n)")
    if confirm_execute == "y":
        asyncio.run(run())