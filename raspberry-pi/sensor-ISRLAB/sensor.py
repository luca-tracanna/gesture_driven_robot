#!./bin/python

import serial
import sys
import paho.mqtt.client as mqtt
from enum import Enum
import json
import time
import numpy as np
import timeit

class Directions(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4

  
    
class SenseController:
    
    def __init__(self, usb_device, baud_rate=9600, broker_address="localhost"):

        self._mqtt_manager = MqttManager(self, broker=broker_address) 
        self._old_sense = dict() 
        self._serial = serial.Serial(usb_device, baud_rate)
        self._serial.flush()

        
        self._sense_list = [200] * 210 
        
    def read_and_publish(self): 

        try:
            current_read = self._serial.readline().decode("ascii").split()
            
            if len(current_read) != 4:
                return print("È stato letto un numero errato di valori")

            angle, dist_left, dist_front, dist_right = current_read
            
            angle = int(angle) - 55
            dist_left = int(dist_left)
            dist_front = int(dist_front)
            dist_right = int(dist_right)


            self._sense_list[angle] = dist_left
            self._sense_list[angle+70] = dist_front
            self._sense_list[angle+140] = dist_right

            current_sense = {
                    Directions.LEFT.name : min(self._sense_list[0:41])/100,
                    Directions.FRONTLEFT.name : min(self._sense_list[42:83])/100,
                    Directions.FRONT.name : min(self._sense_list[84:125])/100,
                    Directions.FRONTRIGHT.name : min(self._sense_list[126:167])/100,
                    Directions.RIGHT.name : min(self._sense_list[168:209])/100
            }


            if current_sense != self._old_sense:
                sense_controller._mqtt_manager._client.publish("/sensors", json.dumps(current_sense))
                print("published :",str(current_sense))
                self._old_sense = current_sense
 
                           
        except Exception as e:
            print("Lettura fallita: "+str(e), flush=True)

class MqttManager:
    _client: mqtt.Client
    _sense_controller: SenseController
    
    def __init__(self, sense_controller: SenseController, username="sense_module", password="sense_module", broker="localhost", port=1883):
        self._sense_controller = sense_controller

        self._client = mqtt.Client(username)
        self._client.username_pw_set(username, password, )
        
        while True:
            try:
                result = self._client.connect(broker, port)
                if result == 0:  # Connessione riuscita
                    print("Connessione riuscita!" +
                          str(self._client.is_connected()))
                    self._client.loop_start()
                    break 
                else:
                    print(
                        f"Tentativo di connessione fallito. Codice di risultato: {result}")
            except Exception as e:
                print(f"Connessione fallita, attendere l'avvio dei container")

            time.sleep(5)
    

if __name__ == "__main__":

    if len(sys.argv) != 3:
        sys.exit("Numero di argomenti errato, specifica l'indirizzo ip del broker e la porta usb dell'arduino")

    sense_controller = SenseController(usb_device=sys.argv[2], broker_address=sys.argv[1])


    while True:
        
        sense_controller.read_and_publish()
           
        if not sense_controller._mqtt_manager._client.is_connected():
            break
        
    print("Closing")
        
    
