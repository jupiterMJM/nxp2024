#!/usr/bin/env python3

"""
Caveat when attempting to run the examples in non-gps environments:

`drone.offboard.stop()` will return a `COMMAND_DENIED` result because it
requires a mode switch to HOLD, something that is currently not supported in a
non-gps environment.

refer to following website to understand the algorithm : https://discuss.px4.io/t/offboard-mode-trajectory-setpoint/32850
"""

import asyncio
import traceback
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import time
import numpy as np
import json

on_drone_reel = False
current_info = list()  # north_m, east_m, down_m, speed
historic = {"north":list(), "east":list(), "down":list(), "speed":list()}

async def move_in_ned_with_velocity(drone:System, aiming_pos, velocity, tolerance=0.2, little_sleep = 0.1):
    """
    aiming_pos: la position RELATIVE au drone actuellement sous (pos_sn, pos_we, pos_hb)
    cad que pour (x, y, z) le drone ira à x mètre vers le QUOI, y metre vers le QUOI, z metre vers le QUOI
    le drone bougera avec une vitesse de velocity
    """
    
    pattern_velocity = lambda x: velocity if x > 1 else 0.5
    # on recupere les coordonnees initiales du drone
    get_position = lambda : current_info[:-1]
    get_speed_norm = lambda : current_info[-1]

    drone_position_init = get_position()
    drone_position_aim = np.array(drone_position_init) + np.array(aiming_pos)
    # on enregistre cette info même si save_traj==False (dans ce cas là, c'est juste que l'info ne sera pas enregistré)
    historic["Info"] = [*drone_position_aim, velocity]
    prev_time = time.time()
    # puis on boucle jusqu'à ce qu'on y soit
    while True:
        drone_position_current = np.array(get_position())
        
        speed = get_speed_norm()
        vecteur_dir = drone_position_aim - drone_position_current
        distance_to_aim = np.linalg.norm(vecteur_dir)
        print(distance_to_aim)
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


async def save_trajectory_in_ned(drone, save_traj=True, keep_one_on=2, backup=100):
    """
    :param: le backup est effectué tous les *backup* iterations
    :param: on garde une donnée sur *keep_one_on*
    """
    global historic, current_info
    indic = 0
    async for position_ned in drone.telemetry.position_velocity_ned():
        indic += 1
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
        


async def run(save_trajectory=True, land_on_point=False):
    """ Does Offboard control using position NED coordinates. """

    drone = System()
    if not on_drone_reel:
        await drone.connect(system_address="udp://:14540")
    else:
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    if save_trajectory:
        saving_traj = asyncio.ensure_future(save_trajectory_in_ned(drone))

    

    print("--Take OFF")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print("[INFO] Wait before offboard")
    await asyncio.sleep(5)

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(*current_info[:-1], 0.0))


    print("-- Starting offboard")
    try:
        await drone.offboard.start()
        print("[INFO] Offboard started, wait...")
        await asyncio.sleep(5)
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    try:
        await move_in_ned_with_velocity(drone, (2, 2, 0), 2)
        await asyncio.sleep(3)
        print("-- Stopping offboard")
        await drone.offboard.stop()
        await drone.action.hold()
    except Exception:
        traceback.print_exc()
    finally:
        if not land_on_point:
            print("[INFO] Returning to launch")
            await drone.action.return_to_launch()
        else: # if land on point
            await drone.action.land()
        
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                await asyncio.sleep(1)
                if save_trajectory:
                    saving_traj.cancel()
                    with open(f"trajectory.json", "w") as f:
                        json.dump(historic, f)
                        print("[INFO] Trajectory saved (backup)")
                print("[INFO] Disarming")
                await drone.action.disarm()
                return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
