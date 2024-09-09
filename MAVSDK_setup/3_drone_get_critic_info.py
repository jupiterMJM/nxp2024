"""
ce programme permet de récolter les données critiques du drone
    tel que: geofence; rc_loss; 
il permet aussi d'envoyer certains ordre au drone
    tel que: killswitch, safe land, return to home
auteurs: Arnaud, Maxence
"""

# importation des modules nécessaires
import asyncio
from mavsdk import System
import mavsdk

drone_reel_horus = True

# obtenir les infos de la geofence
async def get_info_geofence(drone: System):
    """
    NON TESTE
    a pour objectif de récupérer la geofence qui est enregistree sur le drone
    serait utilisé avant le vol, dans les routines de vérifications
    inspiré de: https://docs.px4.io/main/en/config/safety.html
    :comment: l'idee est d'envoyer à la main la commande Mavlink
    ca sera bcp plus complexe si la geofence n'est pas un cylindre
    """
    print(drone.shell.send("PARAM SHOW GF_MAX_HOR_DIST"))   # rayon horizontal de la geofence
    print(drone.shell.send("PARAM SHOW GF_MAX_HOR_DIST"))   # hauteur max de la geofence
    print(drone.shell.send("PARAM SHOW GF_ACTION"))         # l'action associée en cas de violation de la géofence

async def run():
    # Init the drone
    drone = System()
    print("connecting with the drone")
    if drone_reel_horus:
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        await drone.connect()
        print("[WARNING] lancement sur simulateur")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    # Start the tasks
    asyncio.ensure_future(get_info_geofence(drone))

    while True:
        await asyncio.sleep(1)



if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())