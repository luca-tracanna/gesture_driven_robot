from enum import Enum
import paho.mqtt.client as mqtt
import json


class Constants:
    BROKER_HOSTNAME = "mosquitto"  # hostname del broker indicato nel docker-compose.yml
    USERNAME = "perc_module"
    PASSWORD = "perc_module"
    PORT = 1883
    
    LONG_DISTANCE = 0.4
    MEDIUM_DISTANCE = 0.25
    SHORT_DISTANCE = 0.1

class Directions(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    STOP = -1
    
class PerceptionsController:

    def __init__(self, sensors: dict[Directions]):
        self._sensors = sensors
        
        self._old_perceptions = {Directions.LEFT.name: True, Directions.FRONTLEFT.name: True,
                        Directions.FRONT.name: True, Directions.FRONTRIGHT.name: True, Directions.RIGHT.name: True}
        
        self._mqtt_manager = MqttManager(self)
        self._mqtt_manager._client.loop_forever()
        
    def get_free_spaces(self, sensor_data: str):
        
        # Crea il dizionario values
        values = {sensor: sensor_data[sensor.name] for sensor in sensors}

        new_perceptions = {Directions.LEFT.name: True, Directions.FRONTLEFT.name: True,
                        Directions.FRONT.name: True, Directions.FRONTRIGHT.name: True, Directions.RIGHT.name: True}


        # Se uno dei sensori FRONTALI ha un valore minore di LONG_DISTANCE, blocco solo quella direzione
        for sensor in self._sensors:
            if sensor == Directions.LEFT or sensor == Directions.RIGHT:
                continue
            if values[sensor] < Constants.LONG_DISTANCE:
                new_perceptions[sensor.name] = False

        # Se uno dei sensori ha un valore minore di MEDIUM_DISTANCE, blocco quella direzione e le due adiacenti (se esistono entrambe)
        for sensor in self._sensors:
            if sensor == Directions.LEFT or sensor == Directions.RIGHT:
                continue
            if (values[sensor] < Constants.MEDIUM_DISTANCE):
                new_perceptions[sensor.name] = False
                if (sensor.value - 1 >= 0):
                    if(Directions(sensor.value - 1) != Directions.LEFT):
                        new_perceptions[Directions(sensor.value - 1).name] = False
                if (sensor.value + 1 <= 4):
                    if(Directions(sensor.value + 1) != Directions.RIGHT):
                      new_perceptions[Directions(sensor.value + 1).name] = False
                    
                    
        # # Se uno dei sensori ha un valore minore di SHORT_DISTANCE, blocco tutte le direzioni frontali. Se il sensore in questione è diverso da FRONT, blocco anche la direzione laterale (LEFT, RIGHT) più vicina
        for sensor in self._sensors: 
            if values[sensor] < Constants.SHORT_DISTANCE:
                
                new_perceptions[Directions.FRONTLEFT.name] = False
                new_perceptions[Directions.FRONT.name] = False
                new_perceptions[Directions.FRONTRIGHT.name] = False
                
                if sensor == Directions.LEFT or sensor == Directions.FRONTLEFT: new_perceptions[Directions.LEFT.name] = False
                if sensor == Directions.RIGHT or sensor == Directions.FRONTRIGHT: new_perceptions[Directions.RIGHT.name] = False

        return new_perceptions
    
    def publish_free_spaces(self, new_perceptions):
        if (new_perceptions != self._old_perceptions):
            print("Spazi liberi: " + str(new_perceptions), flush=True)
            # Converti il dizionario in formato JSON
            json_message = json.dumps(new_perceptions)

            print("New perceptions: " + str(json_message), flush=True)

            # Invia il messaggio JSON al topic "/perceptions"
            self._mqtt_manager._client.publish("/perceptions", json_message)
            self._old_perceptions = new_perceptions
            
class MqttManager():
    def __init__(self, perceptions_controller: PerceptionsController):
        self._client = mqtt.Client(Constants.USERNAME)
        self._client.username_pw_set(Constants.USERNAME, Constants.PASSWORD)
        self._client.connect(Constants.BROKER_HOSTNAME, Constants.PORT)
        self._client.on_connect = self.on_connect
        self._client.on_message = self.on_message
        self._perceptions_controller = perceptions_controller

    def on_connect(self, client, userdata, flags, rc):
        # flush serve per stampare subito sulla console del docker compose
        print("Connected with result code "+str(rc), flush=True)
        self._client.subscribe("/sensors")
        
    def on_message(self, client, userdata, msg):
        # Spacchetta la stringa JSON
        sensor_data = json.loads(msg.payload.decode())

        # Calcolo le perceptions basandomi sui valori dei sensori
        new_perceptions = self._perceptions_controller.get_free_spaces(sensor_data)
        
        # Verifico se le perceptions sono cambiate e, in tal caso, le pubblico
        self._perceptions_controller.publish_free_spaces(new_perceptions)  

if __name__ == "__main__":
    # Sensori da sinistra a destra
    sensors = [Directions.LEFT, Directions.FRONTLEFT,
                Directions.FRONT, Directions.FRONTRIGHT, Directions.RIGHT]
    perceptions_controller = PerceptionsController(sensors)

    
