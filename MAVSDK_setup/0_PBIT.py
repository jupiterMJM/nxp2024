#!/usr/bin/env python3

import asyncio
from mavsdk import System

drone_reel_horus = True

async def run():
    # Init the drone
    drone = System()
    print("[INFO] Connecting with the drone")
    if drone_reel_horus:
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        await drone.connect()
        print("[WARNING] Lancement sur simulateur")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[INFO] Drone discovered!")
            break
    async for health in drone.telemetry.health():
        #print('[INFO] Test Gyrometer calibration')
        if not health.is_gyrometer_calibration_ok:
            print('[WARNING] Gyrometer calibration not ok')
            
        else :
            print('[INFO] Gyrometer calibration ok')
        #print('[INFO] Test Accelerometer calibration')
        if not health.is_accelerometer_calibration_ok:
            print('[WARNING] Accelerometer calibration not ok')
            
        else :
            print('[INFO] Accelerometer calibration ok')
        #print('[INFO] Test Magnetometer calibration')
        if not health.is_magnetometer_calibration_ok :
            print('[WARNING] Magnetometer calibration not ok')
            
        else :
            print('[INFO] Magnetometer calibration ok')
        #print('[INFO] Test Local Position')
        if not health.is_local_position_ok :
            print('[WARNING] Local Position not good enough to fly')
            
        else :
            print('[INFO] Local Position ok')
        #print('[INFO] Test Global Position')
        if not health.is_global_position_ok :
            print('[WARNING] Global Position not good enough to fly')
            
        else :
            print('[INFO] Global Position ok')
        #print('[INFO] Test Home Position')
        if not health.is_home_position_ok :
            print('[WARNING] Home Position not initialized')
            
        else :
            print('[INFO] Home Position ok')
        #print('[INFO] Test Drone is Armable')
        if not health.is_armable :
            print('[WARNING] Not Armable')
            
        else :
            print('[INFO] Drone is Armable')
        #print('[INFO] Everything is ok')
        break



if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())
