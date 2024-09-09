#Importation des bibliothèques

import asyncio

from mavsdk import System

drone_reel_horus = True

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

    #Armement du drone
    print("[INFO] Arming")
    await drone.action.arm()
    print("[INFO] Moteur Armed")
    await asyncio.sleep(15)
    
    #Désarmement du drone
    print("[INFO] Disarming")
    await drone.action.disarm()
    
    await asyncio.get_event_loop().shutdown_asyncgens()


if __name__ == "__main__":
    # Run the asyncio loop
    print("[INFO] Programme Armement des moteurs")
    confirm_execute = input("Confirmer lancement programme (y/n)")
    if confirm_execute == "y":
        asyncio.run(run())
