#!/usr/bin/env python3

import asyncio

from mavsdk import System
import mavsdk.mission_raw

drone_reel_horus = False

async def run():
    drone = System()
    #Connection avec le drone
    drone = System()
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

    #Calibration de l'altitude du RTL -> hyper important
    print('[INFO] Set Return to launch altitude')
    await drone.action.set_return_to_launch_altitude(1)


    #Tache à executer
    mission_stop_task = asyncio.ensure_future(
        mission_stop(drone)) #voir print_mission_progress

    running_tasks = [mission_stop_task] #running_task contient les différentes tâches qui vont s'executer lors du vol

    #Vérifie si le drone a fini sa mission /!\ ensure_future
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks)) #voir observe_is_in_air

    #Importation de la mission QGroundControl
    mission_import_data = await \
        drone.mission_raw.import_qgroundcontrol_mission(
            "mission_qgc.plan")
    print(f"{len(mission_import_data.mission_items)} mission items imported")
    await drone.mission_raw.upload_mission(mission_import_data.mission_items)
    print("[INFO] Mission uploaded")

    #Armement du drone
    print("[INFO] Arming")
    await drone.action.arm()

    
    #Démarage de la mission
    print("[INFO] Starting mission")
    await drone.mission_raw.start_mission() 

    await termination_task



async def mission_stop (drone : System):
    async for mission_progress in drone.mission_raw.mission_progress():
        print(mission_progress.current, "/", mission_progress.total)
        if mission_progress.current==4: #Stope la mission au 2e item
            print('[INFO] Mission Stop')
            await drone.mission_raw.clear_mission() #Suprime la mission en cours
            await drone.action.hold() #Demande au drone de maintenir sa position GPS
            await asyncio.sleep(10)
            print('[INFO] Return to Launch')
            await drone.action.return_to_launch()


async def observe_is_in_air(drone, running_tasks): #Met fin au programme si le drone est au sol
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False #Avant de décollet, le drone est au sol

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air: # = Si le drone vient d'attérir
            print('[INFO] Drone on Ground')
            for task in running_tasks:
                task.cancel() #Supprime toutes les tâches en cours d'execution
                try: #vérifie si la tâche à bien été supprimée
                    await task 
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens() #Met fin à la boucle asyncio et donc fin au programme

            return


if __name__ == "__main__":
    # Run the asyncio loop
    print("[INFO] Programme Mission QGC avec arret")
    print("[BE CAREFUL] Vérifier la mission avant d'executer")
    confirm_execute = input("Confirmer lancement programme (y/n)")
    if confirm_execute == "y":
        asyncio.run(run())
