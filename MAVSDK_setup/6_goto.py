#!/usr/bin/env python3

import asyncio
from mavsdk import System

#Coordonnées
lat = 47.397606
long = 8.543060

drone_reel_horus = False

async def run():
    global lat, long
    drone = System()
    print("[INFO] Connexion au drone")
    if drone_reel_horus:
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        await drone.connect()
        print("[WARNING] lancement sur simulateur")

    print("[INFO] Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[INFO] Connected to drone!")
            break

    print("[INFO] Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[INFO] Global position state is good enough for flying.")
            break

    print("[INFO] Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print("[INFO] Arming")
    await drone.action.arm()

    print("[INFO] Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)
    # To fly drone 20m above the ground plane
    flying_alt = absolute_altitude + 2
    # goto_location() takes Absolute MSL altitude
    print("[INFO] Starting mooving")
    await drone.action.goto_location(lat, long, flying_alt, 0)

    await asyncio.sleep(5)
    
    print('[INFO] Landing')
    await drone.action.land()
    
    print('[INFO] Fermeture de la connexion')
    await asyncio.get_event_loop().shutdown_asyncgens() #Met fin au programme


if __name__ == "__main__":
    # Run the asyncio loop
    print("[INFO] Programme Goto")
    print("[BE CAREFUL] Vérifier les coordonnées avant de lancer")
    print("[INFO] les coordonnées actuelles sont :", lat, long)
    confirm_execute = input("Confirmer lancement programme (y/n)")
    if confirm_execute == "y":
        asyncio.run(run())
