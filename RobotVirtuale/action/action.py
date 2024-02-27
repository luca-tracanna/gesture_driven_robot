

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import paho.mqtt.client as mqtt
from enum import Enum


class Constants:
    BROKER_HOSTNAME = "mqtt_virtuale" # hostname del broker indicato nel docker-compose.yml
    USERNAME = "action_module"
    PASSWORD = "action_module"
    PORT = 1883 


class Command(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    STOP = -1



class Controller:

    
    def __init__(self, left_motor_name: str, right_motor_name: str):
        
        print("Connecting to simulator...")
        self._cSim_client = RemoteAPIClient(host="host.docker.internal")
        self._sim = self._cSim_client.require('sim')
        print("Connected to SIM")
        
        self._left_motor_handle = self._sim.getObject("./" + left_motor_name)
        self._right_motor_handle = self._sim.getObject("./" + right_motor_name)

    def set_speeds(self, right_speed, left_speed):
        self._sim.setJointTargetVelocity(self._left_motor_handle, left_speed)
        self._sim.setJointTargetVelocity(self._right_motor_handle, right_speed)
      
    def do_action(self, command: Command):
         
        match command:
            case Command.FRONT:
                self.set_speeds(2, 2)
            case Command.FRONTLEFT:
                self.set_speeds(2, 1)
            case Command.FRONTRIGHT:
                self.set_speeds(1, 2)
            case Command.RIGHT:
                self.set_speeds(-0.5, 0.5)
            case Command.LEFT:
                self.set_speeds(0.5, -0.5)
            case Command.STOP:
                self.set_speeds(0, 0)

            case _:
                raise Exception(f"Unknown command {command}")

        print("Azione eseguita: " + command.name, flush=True)


class MqttManager:


    def __init__(self, controller: Controller):
        
        self._controller = controller
        
        self._client = mqtt.Client(Constants.USERNAME)
        self._client.username_pw_set(Constants.USERNAME, Constants.PASSWORD)
        self._client.connect(Constants.BROKER_HOSTNAME, Constants.PORT)
        self._client.on_connect = self.on_connect
        self._client.on_message = self.on_message
        self._client.loop_forever()
        

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc), flush=True)
        self._client.subscribe("/actions")

    def on_message(self, client, rc, msg):
        decoded_msg = msg.payload.decode()
        self._controller.do_action(Command(int(decoded_msg)))


if __name__ == "__main__":
    controller = Controller("leftMotor", "rightMotor")
    manager = MqttManager(controller)