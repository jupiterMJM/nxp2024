#!/usr/bin/env python3

import asyncio

from mavsdk import System
import mavsdk.mission_raw

drone_reel_horus = True

async def run():
    drone = System()
    if drone_reel_horus:
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        await drone.connect()
        print("[WARNING] lancement sur simulateur")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print('[INFO] Importing Mission')
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


if __name__ == "__main__":
    # Run the asyncio loop
    print("[INFO] Programme Mission de QGroundControl")
    print("[BE CAREFUL] Vérifier la mission avant d'executer")
    confirm_execute = input("Confirmer lancement programme (y/n)")
    if confirm_execute == "y":
        asyncio.run(run())
