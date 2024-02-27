"""
    A Class which embeds the Robot SenseController sensors
    in a docker container
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import paho.mqtt.client as paho
import time
import json
from enum import Enum



class Constants:
    BROKER_HOSTNAME = "mqtt_virtuale"
    PORT = 1883
    USERNAME = "sense_module"
    PASSWORD = "sense_module"
    SENSORS = ["ultrasonicSensor[1]",
                    "ultrasonicSensor[2]",
                    "ultrasonicSensor[3]",
                    "ultrasonicSensor[4]",
                    "ultrasonicSensor[5]"]

class Directions(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4

class SenseController:

    def __init__(self, sensors):
        self._sensors = sensors
        self.connect_to_simulator()
        self._old_sense = ""
        
        self._client = paho.Client(Constants.USERNAME)
        self._client.username_pw_set(Constants.USERNAME, Constants.PASSWORD)
        self._client.connect(Constants.BROKER_HOSTNAME, Constants.PORT)
        
    
    def connect_to_simulator(self):
        print("Connecting to simulator...")
        self._cSim_client = RemoteAPIClient(host="host.docker.internal")
        self._sim = self._cSim_client.require('sim')
        print("Connected to SIM")
        self._sensors_handles = [self._sim.getObject("./"+sensor) for sensor in self._sensors]
        self._sim.startSimulation()
        

    def sense(self):
        front_sensor_dists = []
        for handle in self._sensors_handles: #ottengo le distanze dai sensori e filtro eventuali zeri
            dist = self._sim.readProximitySensor(handle)[1]
            if(dist == 0):
                dist = 100
            front_sensor_dists.append(dist)
        
        new_sensors_values = { Directions(i).name : dist  for i, dist in enumerate(front_sensor_dists) }
    
        return json.dumps(new_sensors_values)
    
    def publish_sense(self, new_sense):
        if(self._old_sense != new_sense):
            # print("New sense are: " + new_sense.replace("ultrasonicSensor","sens_id"), flush=True)
            self._client.publish("/sensors", new_sense) # publish message
            self._old_sense = new_sense



if __name__ == "__main__":
    sense_controller = SenseController(Constants.SENSORS)
    
    while(True):
        time.sleep(0.2)
        new_sense = sense_controller.sense()    
        sense_controller.publish_sense(new_sense)
        