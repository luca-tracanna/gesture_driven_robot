from enum import Enum
from json import dumps, loads
import paho.mqtt.client as mqtt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any
import math


class Mode(Enum):
    MANUAL = 1
    AUTO = 2


class Command(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    STOP = -1


class Constants:
    MY_SIM_HOST = "host.docker.internal"
    BROKER_HOSTNAME = "mqtt_virtuale"  # hostname del broker indicato nel docker-compose.yml
    USERNAME = "contr_module"
    PASSWORD = "contr_module"
    PORT = 1883
    CLOSE_ENOUGH_THRESHOLD = 0.05  # m


class Controller:

    def __init__(self):
        self._mode = Mode.MANUAL
        self._free_spaces = dict()
        self._last_action = ""

        # se diversa da "", allora significa che sto in una sessione di obstacle avoidance
        self._last_avoiding_command = ""
        self._mqtt_manager = MqttManager(self)

        # Invio della prima operazione all'avvio (stop)
        self._last_command = Command.STOP
        self.exec_command(Command.STOP)

        # Recupero degli handler dal simulatore
        self.connect_to_sim()

        # Recupero la posizione del robot dal simulatore
        self.update_pos_and_orient()

        # Recupero le posizioni dei targets dal simulatore
        self._targets = dict()
        self.get_targets()

        # Invio della prima operazione all'avvio (stop)
        self._last_command = Command.STOP
        self.exec_command(Command.STOP)

        self._mqtt_manager._client.loop_forever()

    def connect_to_sim(self):
        print("Connecting to simulator...", flush=True)
        client = RemoteAPIClient(host=Constants.MY_SIM_HOST)
        self._sim = client.require('sim')
        print('Connected', flush=True)
        self._robot_handler = self._sim.getObject("./PioneerP3DX")

    def update_pos_and_orient(self):
        robot_x, robot_y, _ = self._sim.getObjectPosition(self._robot_handler)
        # Aggiorno la posizione attuale del robot
        self._my_pos = (robot_x, robot_y)

        # Convertire gli angoli da radianti a gradi
        _, _, self._my_orientation = tuple(
            map(math.degrees, self._sim.getObjectOrientation(self._robot_handler, -1)))
        self._mqtt_manager._client.publish(
            "/position", dumps({"position": self._my_pos, "orientation": self._my_orientation}))

    def get_targets(self):
        for i in range(1, 6):
            # Ottengo la posizione (in metri) degli obiettivi relativa all'origine della scena
            x, y, _ = self._sim.getObjectPosition(
                self._sim.getObject("./Disc"+str(i)), -1)
            # in alcuni casi la coordinata 0 non viene restituita come 0 ma come un numero infinitamente piccolo
            x = 0 if x < 0.00001 and x > -0.0001 else x
            y = 0 if y < 0.00001 and y > -0.0001 else y
            self._targets[str(i)] = x, y

    # ricava l'azione dall'ultimo comando dato e la pubblica se è diversa da quella attuale

    def exec_command(self, command):

        # pubblicazione del messaggio solo se la nuova azione è diversa da quella attuale
        if (self._last_action != command):
            self._mqtt_manager._client.publish("/actions", command.value)
            self._last_action = command

    def check_free_spaces(self, cmnd1: Command, cmnd2: Command, cmnd3: Command, cmnd4: Command):
        ##
        # In questa funzione vengono passati i comandi nell'ordine in cui voglio che siano valutati
        # Ogni comando ha un ordine diverso, per questo motivo è necessario parametrizzare.
        # Se il comando è front, allora controllo frontLeft, frontRight, left e right
        # Se il comando è frontLeft, allora controllo Left, front, frontRight e right (speculare per frontRight)
        # Se il comando è left, allora controllo frontLeft, front, frontRight e right (speculare per right)
        ##

        sorted_commands = [cmnd1, cmnd2, cmnd3, cmnd4]
        for cmnd in sorted_commands:
            if (self._free_spaces[cmnd]):
                self._last_avoiding_command = cmnd
                return cmnd
        return Command.STOP

    def avoid_obstacles(self):

        # L'obstacle avoidance non viene fatta se l'ultimo comando è STOP
        if (self._last_command == Command.STOP):
            return self._last_command

        # L'obstacle avoidance non viene fatta se gli spazi vuoti non sono inizializzati
        if (not self._free_spaces):
            return self._last_command

        # L'obstacle avoidance non viene fatta se la direzione in cui sto andando è libera
        if (self._free_spaces[self._last_command]):
            self._last_avoiding_command = ""
            return self._last_command

        command = self._last_command
        # Se l'avoiding command è diverso da "", vuol dire che mi trovo nella stessa "sessione" di obstacle avoidance, quindi se
        # quella direzione è libera continuo a seguirla
        if self._last_avoiding_command != "":
            if self._free_spaces[self._last_avoiding_command]:
                return self._last_avoiding_command

            command = self._last_avoiding_command

        # Se arrivo qui vuol dire che command è occupato
        match command:
            case Command.FRONT:
                return self.check_free_spaces(Command.FRONTLEFT, Command.FRONTRIGHT, Command.LEFT, Command.RIGHT)

            case Command.FRONTLEFT:
                if (self._mode == Mode.MANUAL):
                    return self.check_free_spaces(Command.LEFT, Command.FRONT, Command.FRONTRIGHT, Command.RIGHT)
                return self.check_free_spaces(Command.FRONT, Command.FRONTRIGHT, Command.RIGHT, Command.LEFT)

            case Command.FRONTRIGHT:
                if (self._mode == Mode.MANUAL):
                    return self.check_free_spaces(Command.RIGHT, Command.FRONT, Command.FRONTLEFT, Command.LEFT)
                return self.check_free_spaces(Command.FRONT, Command.FRONTLEFT, Command.LEFT, Command.RIGHT)

            case Command.LEFT:
                return self.check_free_spaces(Command.FRONTLEFT, Command.FRONT, Command.FRONTRIGHT, Command.RIGHT)

            case Command.RIGHT:
                return self.check_free_spaces(Command.FRONTRIGHT, Command.FRONT, Command.FRONTLEFT, Command.LEFT)

    def get_dir_to_target(self, pos_to_reach):
        x_to_reach, y_to_reach = pos_to_reach
        my_x, my_y = self._my_pos

        # L'angolo da - FRONT_ANGLE a FRONT_ANGLE viene considerato come dritto
        FRONT_ANGLE = 5
        # L'angolo da - FRONT_SIDE_ANGLE a FRONT_SIDE_ANGLE viene considerato come quasi dritto, quindi il robot utilizzerà le operazioni frontSide (frontLeft, frontRight)
        FRONT_SIDE_ANGLE = 60

        # Calcolo la direzione tra la posizione del robot e il target
        direction_to_reach = math.degrees(
            math.atan2(y_to_reach - my_y, x_to_reach - my_x))

        # aggiungiamo 540 perché in python il modulo di un numero negativo non funziona come vorremmo
        variation = (direction_to_reach -
                     self._my_orientation + 540) % 360 - 180

        # In base all'ampiezza della variazione decido il comando da eseguire
        assert -180 < variation <= 180

        if variation >= FRONT_SIDE_ANGLE:
            self._last_command = Command.LEFT
        elif variation > FRONT_ANGLE and variation < FRONT_SIDE_ANGLE:
            self._last_command = Command.FRONTLEFT
        elif variation < -FRONT_ANGLE and variation > -FRONT_SIDE_ANGLE:
            self._last_command = Command.FRONTRIGHT
        elif variation <= -FRONT_SIDE_ANGLE:
            self._last_command = Command.RIGHT
        else:
            # Consideriamo la direzione dritta anche se c'è un piccolo scarto di +-5 gradi
            self._last_command = Command.FRONT

    def close_enough(self, pos_to_reach):
        x_to_reach, y_to_reach = pos_to_reach
        my_x, my_y = self._my_pos
        # Controllo che la distanza sia minore del threashold
        return (x_to_reach - my_x) ** 2 + (y_to_reach - my_y) ** 2 <= Constants.CLOSE_ENOUGH_THRESHOLD

    def closer_than_obstacle(self, pos_to_reach):
        OBSTACLE_DISTANCE = 0.6
        x_to_reach, y_to_reach = pos_to_reach
        my_x, my_y = self._my_pos
        return (x_to_reach - my_x) ** 2 + (y_to_reach - my_y) ** 2 <= OBSTACLE_DISTANCE

    def handle_manual_cmnd(self, decoded_msg: str):
        # Converto il comando da stringa a Enum
        decoded_msg = Command(int(decoded_msg))

        # Imposto last_command come il comando dato e resetto last_avoiding_command in modo da interrompere un'eventuale sessione di avoiding
        self._last_command = decoded_msg
        self._last_avoiding_command = ""

        # Verifico se bisogna fare eventuali avoiding
        command = self.avoid_obstacles()

        # Eseguo il comando calcolato
        self.exec_command(command)

    def handle_auto_cmnd(self, decoded_msg: str):
        if (decoded_msg == str(Command.STOP.value)):
            self._last_command = Command.STOP
            self.exec_command(Command.STOP)

        else:
            # Recupero dal dizionario la posizione da raggiungere
            pos_to_reach = self._targets[decoded_msg]

            # Se mi sono avvicinato abbastanza all'obiettivo, mi fermo
            if (self.close_enough(pos_to_reach)):

                self._last_command = Command.STOP
                self.exec_command(Command.STOP)
                Command.FRONT
                # Avviso il modulo gesture che sono arrivato
                self._mqtt_manager._client.publish(
                    "/gesture_confirm", "reached")

            else:

                # Imposta last command come la direzione migliore per avvicinarsi al target
                self.get_dir_to_target(
                    pos_to_reach)

                # Su tale direzione eseguo l'obstacle avoidance
                command = self.avoid_obstacles()

                # Eseguo il comando così calcolato
                self.exec_command(
                    command)

    def handle_perceptions(self, decoded_msg: str):
        # Aggiorno i freespaces
        decoded_json = loads(decoded_msg)

        # Dato che il json.loads converte le chiavi in stringhe, le trasformo in Command
        for key, value in decoded_json.items():
            enum_key = Command[key]
            self._free_spaces[enum_key] = value

        # Verifico se con le nuove perceptions c'è bisogno di evitare un ostacolo
        # command = Command.STOP
        # if (self._controller._last_command != Command.STOP):
        command = self.avoid_obstacles()

        # Eseguo l'operazione
        self.exec_command(command)

    def change_mode(self, decoded_msg: str):
        self._mode = Mode[decoded_msg]
        print("Cambio modalità in " + str(self._mode.name), flush=True)


class MqttManager:

    _controller = Any
    _client = Any

    def __init__(self, controller: Controller):
        self._client = mqtt.Client(Constants.USERNAME)
        self._client.username_pw_set(Constants.USERNAME, Constants.PASSWORD)
        self._client.connect(Constants.BROKER_HOSTNAME, Constants.PORT)
        self._client.on_connect = self.on_connect
        self._client.on_message = self.on_message
        self._controller = controller

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc), flush=True)
        self._client.subscribe("/commands_manual")
        self._client.subscribe("/commands_auto")
        self._client.subscribe("/perceptions")
        self._client.subscribe("/mode")

    def on_message(self, client, rc, msg):
        topic = msg.topic
        decoded_msg = msg.payload.decode()

        self._controller.update_pos_and_orient()

        if (decoded_msg == ""):
            return print("Messaggio vuoto")

        # Se ricevo un messaggio sul topic mode vuol dire che ho cambiato modalità
        if (topic == "/mode"):
            self._controller.change_mode(decoded_msg)

        if (topic == "/commands_manual"):
            self._controller.handle_manual_cmnd(decoded_msg)

        if (topic == "/commands_auto"):
            self._controller.handle_auto_cmnd(decoded_msg)

        if (topic == "/perceptions"):
            self._controller.handle_perceptions(decoded_msg)


if __name__ == "__main__":
    controller = Controller()
