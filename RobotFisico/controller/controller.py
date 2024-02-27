from enum import Enum
from json import loads, dumps
from paho.mqtt.client import Client
from math import cos, radians, sqrt, copysign, degrees, atan2


class Mode(Enum):
    MANUAL = 1
    AUTO = 2


class Command(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    SLOW_LEFT = 5
    SLOW_RIGHT = 6
    STOP = -1


class Constants:
    ROBOT_RADIUS = 0.2
    BROKER_HOSTNAME = "mosquitto"  # hostname del broker indicato nel docker-compose.yml
    USERNAME = "contr_module"
    PASSWORD = "contr_module"
    PORT = 1883
    LONG_DISTANCE = 0.4  # IMPORTANTE: deve essere uguale alla long distance nel perception


class Controller:

    def __init__(self, targets: dict, arena_width: float = 2, arena_height: float = 2, space_between_tags: float = 0.5):
        self._mode = Mode.MANUAL
        self._free_spaces = dict()
        self._last_action = ""
        self._no_visible_tags = False
        self._reached = False
        # se diversa da "", allora significa che sto in una sessione di obstacle avoidance
        self._last_avoiding_command = ""
        self._mqtt_manager = MqttManager(self)

        self._arena_size = (arena_width, arena_height)
        self._space_between_tags = space_between_tags

        # Inizializzo il dizionario dei tag
        self._tags = []
        i = arena_height/2
        while i >= -arena_height/2:
            j = -arena_height/2
            while j <= arena_height/2:
                self._tags.append({
                    'x': j,
                    'y': i,
                    'dist': 0,
                    'yaw': 0,
                    'phi': 0,
                    'is_visible': False
                })
                j += self._space_between_tags
            i -= self._space_between_tags

        # for tag in enumerate(self._tags):
        #     print(tag, flush=True)

        self._my_pos = (0, 0)
        self._my_orientation = 0

        self._targets = targets

        # Invio della prima operazione all'avvio (stop)
        self._last_command = Command.STOP
        self.exec_command(Command.STOP)

        self._mqtt_manager._client.loop_forever()

    def update_pos_and_orient(self):

        # Restituisce il tag visibile
        tag = [tag for tag in self._tags if tag['is_visible'] == True][0]

        ##
        # Calcolo posizione della camera
        ##

        # Aggiungo la distanza tra la fotocamera e il centro del robot
        if (tag['dist'] == None):
            return

        dist = round(tag['dist'], 3)
        yaw = round(tag['yaw'], 3)
        camera_x = tag['x']
        camera_y = tag['y']

        # Calcolo dell'angolo vicino all'origine del triangolo rettangolo
        beta = (abs(yaw) % 90)
        if (beta != 0):
            # Calcolo dei cateti
            a = dist * cos(radians(beta))
            b = sqrt(dist ** 2 - a ** 2)
            if (yaw >= -90 and yaw <= 90):
                camera_x += copysign(1, yaw) * b
                camera_y -= a
            else:
                camera_x += copysign(1, yaw) * a
                camera_y += b
        else:
            # tratto separatamente i casi limite
            if (yaw == 0):
                camera_y -= dist
            elif (yaw == 90):
                camera_x += dist
            elif (abs(yaw) == 180):
                camera_y += dist
            else:
                camera_x -= dist

        camera_x = round(camera_x, 3)
        camera_y = round(camera_y, 3)

        ##
        # Calcolo orientamento
        ##
        my_or = 0
        phi = tag['phi']
        gamma = abs(yaw)
        if (abs(yaw) % 90 != 0):
            if (gamma > 90):
                gamma = 180 - gamma
            theta = 90 - gamma

            if (yaw > -180 and yaw < -90):
                my_or = -theta - phi
            elif (yaw > 90 and yaw < 180):
                my_or = -180 + theta - phi
            elif (yaw > -90 and yaw < 0):
                my_or = theta - phi
            elif (yaw > 0 and yaw < 90):
                my_or = 180 - theta - phi
            else:
                print("Caso errato, verificare il problema", flush=True)
        # tratto separatamente i casi limite
        else:
            if (yaw == 0):
                my_or = 90 - phi
            elif (yaw == 90):
                # a seconda del segno di phi parto da 180 o -180
                my_or = phi/abs(phi) * 180 - phi
            elif (yaw == 180 or yaw == -180):
                my_or = -90 - phi
            elif (yaw == -90):
                my_or = -phi
            else:
                print(
                    "Caso limite errato, verificare il problema", flush=True)

        my_or = round(my_or, 3)
        self._my_orientation = my_or
        ##
        # Una volta calcolata la posizione della camera, la traslo rispetto alla distanza del robot
        ##
        vertical_leg = 0
        # Il segno per cui deve essere moltiplicato horizontal_leg prima di essere sommato alla x
        horizontal_sign = 1

        if (abs(my_or) == 90):
            self._my_pos = (camera_x, camera_y +
                            copysign(1, my_or) * Constants.ROBOT_RADIUS)

        elif (abs(my_or) == 180):
            self._my_pos = (camera_x - Constants.ROBOT_RADIUS, camera_y)

        elif (my_or == 0):
            self.my_pos = self._my_pos = (
                camera_x + Constants.ROBOT_RADIUS, camera_y)
            return

        elif (my_or < -90 and my_or > -180):
            vertical_leg = Constants.ROBOT_RADIUS * \
                cos(radians(abs(my_or) - 90))  # Segno positivo

        elif (my_or < 0 and my_or > -90):
            vertical_leg = Constants.ROBOT_RADIUS * \
                cos(radians(90 - abs(my_or)))  # Segno positivo
            horizontal_sign = -1

        elif (my_or > 0 and my_or < 90):
            vertical_leg = -1 * Constants.ROBOT_RADIUS * \
                cos(radians(90 - my_or))  # Segno negativo
            horizontal_sign = -1

        elif (my_or > 90 and my_or < 180):
            vertical_leg = -1 * Constants.ROBOT_RADIUS * \
                cos(radians(my_or - 90))  # Segno negativo

        horizontal_leg = horizontal_sign * \
            sqrt(Constants.ROBOT_RADIUS ** 2 - vertical_leg ** 2)
        self._my_pos = (round(camera_x + horizontal_leg, 3),
                        round(camera_y + vertical_leg, 3))

        # print("New position: " + str(self._my_pos) +
        #       ", new orientation: "+str(self._my_orientation), flush=True)

        # Comunico a gesture la nuova posizione in modo da stamparla sull'interfaccia
        self._mqtt_manager._client.publish(
            "/position", dumps({"position": self._my_pos, "orientation": self._my_orientation}))

    def exec_command(self, command: Command):

        # pubblicazione del messaggio solo se la nuova azione è diversa da quella attuale
        if (self._last_action != command):
            print("Azione eseguita: " + command.name, flush=True)
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
        direction_to_reach = degrees(
            atan2(y_to_reach - my_y, x_to_reach - my_x))

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
        return sqrt((x_to_reach - my_x) ** 2 + (y_to_reach - my_y) ** 2) <= Constants.ROBOT_RADIUS/2

    def closer_than_obstacle(self, pos_to_reach):
        x_to_reach, y_to_reach = pos_to_reach
        my_x, my_y = self._my_pos
        return (x_to_reach - my_x) ** 2 + (y_to_reach - my_y) ** 2 <= Constants.LONG_DISTANCE

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
            # Se non vedo tags e non sono arrivato ancora all'obiettivo, allora non eseguo il comando
            if (self._no_visible_tags and not self._reached):
                return print(
                    "Non vedo tags, attendi...", flush=True)
            
            # Se avevo raggiunto l'obiettivo e sono qui, significa che ho ricevuto un nuovo obiettivo
            if(self._reached): self._reached = False

            # Recupero dal dizionario la posizione da raggiungere
            pos_to_reach = self._targets[decoded_msg]

            # Se mi sono avvicinato abbastanza all'obiettivo, mi fermo
            if (self.close_enough(pos_to_reach)):

                self._last_command = Command.STOP
                self.exec_command(Command.STOP)
                self._reached = True

                # Avviso il modulo gesture che sono arrivato
                self._mqtt_manager._client.publish(
                    "/gesture_confirm", "reached")
                print("\nReached\n", flush=True)

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
        command = self.avoid_obstacles()

        # Eseguo l'operazione
        self.exec_command(command)


    def handle_tags(self, decoded_msg):
        # Se sono in modalità manuale non ho bisogno della posizione e dell'orientamento
        if (self._mode == Mode.MANUAL):
            self._no_visible_tags = False
            return

        decoded_tag = loads(decoded_msg)

        # Se non vedo tag, allora ruoto lentamente per trovare i marker nella direzione più prossima a quella in cui stavo andando
        if (int(decoded_tag["ID"]) == -1):

            # Controllo qual è l'ultimo comando che stavo eseguendo
            command = self._last_command
            if (self._last_avoiding_command != ""):
                command = self._last_avoiding_command

    
            if (self._reached):
                self.exec_command(Command.STOP)
            # Se l'ultimo comando è verso destra, ruoto lentamente verso destra
            elif (command == Command.RIGHT or command == Command.FRONTRIGHT):
                self.exec_command(Command.SLOW_RIGHT)
            # Altrimenti ruoto lentamente verso sinistra
            else:
                self.exec_command(Command.SLOW_LEFT)

            # Tramite questa variabile, se arrivano altri comandi so già che non posso eseguirli perché non vedo tag
            self._no_visible_tags = True
            return

        # Se arrivo qui vuol dire che vedo un tag
        if(self._no_visible_tags):
            self.exec_command(Command.STOP)
            self._no_visible_tags = False

        # Se invece vedo marker, aggiorno il dizionario
        self.update_tags(decoded_tag)        

        # Con i dati aggiornati posso aggiornare posizione ed orientamento
        self.update_pos_and_orient()

    def change_mode(self, decoded_msg: str):
        self._mode = Mode[decoded_msg]
        print("Cambio modalità in " + str(self._mode.name), flush=True)
        
    def update_tags(self, received_tag):
        new_tags_list = []
        for i, tag in enumerate(self._tags):
            new_tag = {'x': tag['x'], 'y': tag['y'], 'dist': None,
                       'yaw': None, 'phi': None, 'is_visible': False}
            if i == received_tag['ID']:
                new_tag['dist'] = received_tag['dist']
                new_tag['yaw'] = received_tag['yaw']
                new_tag['phi'] = received_tag['phi']
                new_tag['is_visible'] = True
                # print(
                #     f"\n\n ID:{str(received_tag['ID'])}, dist: {str(new_tag['dist'])}\n\n", flush=True)
            new_tags_list.append(new_tag)

        self._tags = new_tags_list


class MqttManager:

    def __init__(self, controller: Controller):
        self._client = Client(Constants.USERNAME)
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
        self._client.subscribe("/tags")
        self._client.subscribe("/mode")

    def on_message(self, client, rc, msg):
        topic = msg.topic
        decoded_msg = msg.payload.decode()
        # print("Received message: " + str(decoded_msg) + ", Last command: " + str(self._controller._last_command), flush=True)

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

        if (topic == "/tags"):
            self._controller.handle_tags(decoded_msg)


if __name__ == "__main__":
    targets = {'1': (0.5, 0.5), '2': (
        0.5, -0.5), '3': (-0.5, -0.5), '4': (0, 0), '5': (-0.5, 0.5)}
    controller = Controller(targets, 1.5, 1.5)
