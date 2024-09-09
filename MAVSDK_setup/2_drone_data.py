#!/usr/bin/env python3

import asyncio
from mavsdk import System

drone_reel_horus = True

async def run():
    # Init the drone
    drone = System()
    if drone_reel_horus:
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        await drone.connect()
        print("[WARNING] lancement sur simulateur")

    # Start the tasks
    asyncio.ensure_future(print_altitude_lidar(drone))
    asyncio.ensure_future(print_altitude_manometre(drone))

    while True:
        await asyncio.sleep(1)


async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        print(f"Battery: {battery.remaining_percent}")


async def print_gps_info(drone):
    async for gps_info in drone.telemetry.gps_info():
        print(f"GPS info: {gps_info}")

async def print_altitude_lidar(drone):
    async for distance in drone.telemetry.distance_sensor():
        for i in range(2):
            print(f"distance %d: {distance.current_distance_m}"%i)
            
async def print_altitude_manometre(drone):
    async for elt in drone.telemetry.altitude():
        print(f"altitude: {elt}")

if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())
