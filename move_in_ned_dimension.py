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


async def move_in_ned_with_velocity(drone:System, aiming_pos, velocity, tolerance=0.2, save_trajectory=True):
    """
    aiming_pos: la position RELATIVE au drone actuellement sous (pos_sn, pos_we, pos_hb)
    cad que pour (x, y, z) le drone ira à x mètre vers le QUOI, y metre vers le QUOI, z metre vers le QUOI
    le drone bougera avec une vitesse de velocity
    """
    pattern_velocity = lambda x: velocity if x > 1 else 0.5
    # on recupere les coordonnees initiales du drone
    async for position_ned in drone.telemetry.position_velocity_ned():
        drone_position_init = position_ned.position
        break
    historic = {"north":list(), "east":list(), "down":list(), "speed":list()}
    drone_position_aim = (drone_position_init.north_m + aiming_pos[0], drone_position_init.east_m + aiming_pos[1], drone_position_init.down_m + aiming_pos[2])
    
    prev_time = time.time()
    # puis on boucle jusqu'à ce qu'on y soit
    async for position_ned in drone.telemetry.position_velocity_ned():
        drone_position_current = position_ned.position
        
        speed = np.array([position_ned.velocity.north_m_s, position_ned.velocity.east_m_s, position_ned.velocity.down_m_s])
        drone_position_current = np.array([drone_position_current.north_m, drone_position_current.east_m, drone_position_current.down_m])
        vecteur_dir = np.array([drone_position_aim[i] - drone_position_current[i] for i in range(3)])
        distance_to_aim = np.linalg.norm(vecteur_dir)
        print(np.linalg.norm(vecteur_dir))
        if distance_to_aim < tolerance:
            await drone.offboard.set_position_ned(PositionNedYaw(drone_position_aim[0], drone_position_aim[1], drone_position_aim[2], 0.0))
            print("[INFO] Should be arrived")
            await asyncio.sleep(3)
            if save_trajectory:
                with open(f"trajectory.json", "w") as f:
                    historic["Info"] = [*drone_position_aim, velocity]
                    json.dump(historic, f)
                print("[INFO] Trajectory saved")
            break
        vecteur_unit = vecteur_dir / np.linalg.norm(vecteur_dir)

        # puis on dirige le groupe
        dt = time.time() - prev_time
        next_position = drone_position_current + vecteur_unit * pattern_velocity(distance_to_aim)
        await drone.offboard.set_position_ned(PositionNedYaw(next_position[0], next_position[1], next_position[2], 0.0))
        prev_time = time.time()

        if save_trajectory:
            historic["north"].append(position_ned.position.north_m)
            historic["east"].append(position_ned.position.east_m)
            historic["down"].append(position_ned.position.down_m)
            historic["speed"].append(np.linalg.norm(speed))





async def run():
    """ Does Offboard control using position NED coordinates. """

    drone = System()
    await drone.connect(system_address="udp://:14540")

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

    print("--Take OFF")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    try:
        await move_in_ned_with_velocity(drone, (-4, 2, -3), 10)
        await asyncio.sleep(3)
        print("-- Stopping offboard")
        await drone.offboard.stop()
    except Exception:
        traceback.print_exc()
    finally:
        print("[INFO] Returning to launch")
        await drone.action.return_to_launch()
        
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("[INFO] Disarming")
                await drone.action.disarm()
                return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())