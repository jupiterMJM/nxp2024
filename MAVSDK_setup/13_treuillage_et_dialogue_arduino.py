import serial
import time
import asyncio
from mavsdk import System


startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False
debuggage = True
on_drone_reel = True

#========================
#========================
    # the functions

def setupSerial(baudRate, serialPortName):
    
    global  serialPort
    
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)

    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))

   #waitForArduino()

#========================

def sendToArduino(stringToSend):
    
        # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort
    
    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3


#==================

def recvLikeArduino():

    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") # decode needed for Python3
        
        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True
    
    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX" 

#==================

def waitForArduino():

    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    
    print("Waiting for Arduino to reset")
     
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'): 
            print(msg)



#====================
#====================
    # the program

async def lancement_treuillage():
    print("lancement treuillage active")
    setupSerial(115200, "/dev/ttyAMA3")
    prevTime = time.time()
    mission_lancee = False
    try:
        while True:
                    # check for a reply
            arduinoReply = recvLikeArduino()
            if not (arduinoReply == 'XXX'):
                print ("Time %s  Reply %s" %(time.time(), arduinoReply))
                if arduinoReply ==  "statut mission finie":
                    break
                # send a message at intervals

            messages_arduino = arduinoReply.split(" ")
            if len(messages_arduino) > 1:
                if messages_arduino[1] == "ok":
                    mission_lancee = True
                    
                    
            if (time.time() - prevTime > 1.0) and (mission_lancee == False):
                sendToArduino("cmd mission_largage")
                prevTime = time.time()

        print("fin de communication")
    except:
        while True:
            sendToArduino("sec kill")
            
    finally:
        print("fin de mission")
        

async def run(): #Fonction principale
    drone = System()
    #Connection avec le drone
    if on_drone_reel:
        print("[INFO] Connection sur le drone réel!!!")
        await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    else:
        print("[INFO] Connection sur simulation")
        await drone.connect()
    if not debuggage:
        print("[INFO] Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"[INFO] Connected to drone!")
                break

            #Programation de l'altitude voulue pour le décollage
        await drone.action.set_takeoff_altitude(2)
        
        print("[INFO] Arming")
        await drone.action.arm()
        
        #Décollage du drone
        print("[INFO] Taking off")
        await drone.action.takeoff()

        #Test de hold dans le take_off, partie a cammenter si besoin
        ##################################################################
        await asyncio.sleep(10)
        print("[INFO] Passage en hold")
        await drone.action.hold()
        ##################################################################
    print("top")
    await asyncio.sleep(60)
    await lancement_treuillage()

    if not debuggage:
        #Atterissage
        print("[INFO] Landing")
        await drone.action.land()


if __name__ == "__main__":
    # Run the asyncio loop
    print("[INFO] Programme de test de treuillage")
    confirm_execute = input("Confirmer lancement programme (y/n)")
    if confirm_execute == "y":
        asyncio.run(run())
