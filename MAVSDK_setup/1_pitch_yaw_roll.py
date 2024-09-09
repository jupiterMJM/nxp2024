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
        print("[WARNING] lancement sur simulateur")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[INFO] Drone discovered!")
            break
    asyncio.ensure_future(print_pitch(drone))

    while True:
        await asyncio.sleep(1)


async def print_pitch(drone):
    async for elt in drone.telemetry.attitude_euler():
        print(f"Attitude: {elt} rad")

if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())
