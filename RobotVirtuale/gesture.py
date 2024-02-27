import paho.mqtt.client as mqtt
import cv2
import mediapipe as mp
import time
import math
from enum import Enum
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from json import loads

class Command(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    STOP = -1

class Mode(Enum):
    MANUAL = 1
    AUTO = 2

class Constants:
    # Numero di elementi utilizzati per la media
    MEAN_DIMENSION = 20
    # Somma di tutti gli elementi usati per la media pesata
    MEAN_DENOM = sum(range(1, MEAN_DIMENSION + 1))
    # Rate di refresh della camera
    CAM_REFRESH_TIME = 0.05
    # Dimensioni della finestra con la webcam
    WINDOW_WIDTH = 500

class MathUtils:

    @staticmethod
    def get_slope(point_1, point_2):
        slope = 300  # valore molto alto per simulare l'infinito

        if point_1[0] - point_2[0] != 0:
            slope = (point_1[1] - point_2[1]) / (point_1[0] - point_2[0])

        return slope

    @staticmethod
    def distance(point_1, point_2):

        return math.sqrt((point_2[0] - point_1[0])**2 + (point_2[1] - point_1[1])**2)

class handTracker():
    def __init__(self, mode=False, maxHands=1, detectionCon=0.5, modelComplexity=1, trackCon=0.5):
        # Inizializzazione del tracker con i parametri forniti
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.current_lm_list = []

        # Inizializzazione di Mediapipe per il rilevamento delle mani
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def handsFinder(self, image, draw=True):
        # Converte l'immagine da BGR a RGB
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Processa l'immagine per rilevare le mani
        self.results = self.hands.process(imageRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                # Disegna i landmark e le connessioni delle mani sull'immagine
                if draw:
                    self.mpDraw.draw_landmarks(
                        image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image

    def compute_landmarks(self, image):
        # A ogni iterazione svuoto la lista dei landmarks
        self.current_lm_list = []
        
        # Trova e disegna le posizioni dei landmark delle mani sull'immagine
        self.positionFinder(image)

    # restituisce la lista di landmarks sulla mano
    def positionFinder(self, image, handNo=0, draw=True):

        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(Hand.landmark):
                # Ottiene le coordinate normalizzate dei landmark e le converte in pixel
                h, w, _ = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.current_lm_list.append([id, cx, cy])
                # Disegna un cerchio intorno al landmark sull'immagine
                if draw:
                    cv2.circle(image, (cx, cy), 10, (255, 0, 255), cv2.FILLED)
                    # Scrive l'ID del landmark all'interno del cerchio
                    cv2.putText(image, str(id), (cx - 5, cy + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def get_lm_coords(self, id):
        return (self.current_lm_list[id][1], self.current_lm_list[id][2])

class GestureController:

    def __init__(self):
        self._current_mode = Mode.MANUAL
        self._mqtt_manager = MqttManager(self)

        self._tracker = handTracker()

        self._changing_mode = False
        self._mean_counter = 0
        self._meanPos = 0
        
        self._last_operation = ""
        self._reached = False
        self._counter = 0
        self._image = None
          
        self._pos = (0,0)
        self._orient = 0

    def get_index_direction(self):

        wirst_x, wirst_y = self._tracker.get_lm_coords(0)
        index_x, index_y = self._tracker.get_lm_coords(8)
        
        slope = MathUtils.get_slope((wirst_x, wirst_y), (index_x, index_y))

        # Se l'indice si trova in posizione più bassa rispetto al polso, cambio modalità
        if index_y > wirst_y:
            if slope < -2 or slope > 2:
                return -1
            else:
                return Command.STOP

        # Se il coefficiente angolare è compreso tra 1 e -1, differenzio tra sinistra e destra in base a quale x è maggiore tra polso e indice
        if slope < 1 and slope > -1:
            if index_x > wirst_x:
                return Command.RIGHT
            return Command.LEFT

        if slope > 1 and slope < 3:
            return Command.FRONTLEFT

        if slope < -3 or slope > 3:
            return Command.FRONT

        # L'ultimo caso rimasto è slope > -3 and slope < -1
        return Command.FRONTRIGHT

    def get_hand_mean(self):
        ##
        # Essendo l'apertura della mano un'operazione potenzialmente lenta, utilizziamo una media pesata in cui i valori finali hanno un peso più elevato
        # in questo modo si da più importanza ai valori alla fine dell'apertura, che deve impiegare non più di MEAN_DIMENSION/CAM_REFRESH_TIME millisecondi
        ##

        # Caso in cui avevo finito di calcolare la posizione oppure avevo interrotto il calcolo
        if (self._mean_counter == 0 or self._mean_counter == -1):
            self._mean_counter = 1
            self._mean_pos = int(0)
            return Command.STOP

        # Caso in cui sto calcolando la media
        if (self._mean_counter < Constants.MEAN_DIMENSION and self._mean_counter > 0):
            self._mean_pos += self.calculate_number() * self._mean_counter
            self._mean_counter += 1
            return "Calcolando"

        # Caso in cui sono arrivato alla fine del calcolo della media e posso restituire la posizione
        if (self._mean_counter == Constants.MEAN_DIMENSION):
            self._mean_pos = str(
                round(int(self._mean_pos) / Constants.MEAN_DENOM))
            self._mean_counter += 1
            return str(self._mean_pos)

        # Caso in cui ho finito di calcolare la media ma ho ancora la mano aperta (anche se indico un altro numero, viene mantenuto quello calcolato)
        # l'unico modo per ricalcolare un nuovo numero è chiudere la mano
        return str(self._mean_pos)

    def change_mode(self):
        self._changing_mode = True
        if (self._current_mode == Mode.MANUAL):
            self._current_mode = Mode.AUTO
        else:
            self._current_mode = Mode.MANUAL
            self._mean_counter = 0
            self._mean_pos = Command.STOP

        print("Changing Mode in " + self._current_mode.name)
        self._mqtt_manager._client.publish("/mode", self._current_mode.name)

    def compute_operation(self):

        # Se l'indice si trova sotto al polso, allora cambio modalità
        if (self._tracker.current_lm_list and self.get_index_direction() == -1):
            if (not self._changing_mode):
                self.change_mode()
            return Command.STOP
        
        else:
            # se "torno" con l'indice sopra al polso cambiare un'altra volta modalità
            if (self._changing_mode):
                self._changing_mode = False

        ##
        # Se sono in modalità manuale
        ##
        
        if self._current_mode == Mode.MANUAL:
            # Se faccio il pugno in modalità manuale oppure rimuovo le mani dalla finestra il robot si ferma
            if (not self._tracker.current_lm_list or self.calculate_number() == 0):
                return Command.STOP

            # Sennò calcolo e restituisco la direzione
            return self.get_index_direction()
        
        ##
        # Se sono in modalità automatica
        ##
        
        # Se chiudo il pugno o rimuovo le mani dalla finestra ho tre possibilità
        if (not self._tracker.current_lm_list or self.calculate_number() == 0):
            
            # Se ho raggiunto il target e ho la mano chiusa o fuori dall'inquadratura scrivo arrivato 
            if(self._reached):
                return "Arrivato"
            
            # Se torno al pugno chiuso quando ho già finito di calcolare la posizione, continuo a mantenere la posizione calcolata
            if (self._mean_counter == Constants.MEAN_DIMENSION + 1 or self._mean_counter == -1):
                self._mean_counter = -1
                return str(self._mean_pos)

            # Se chiudo la mano prima che la posizione sia stata calcolata, azzero il calcolo
            self._mean_counter = 0
            return Command.STOP

        # Se ho raggiunto il target e ho la mano aperta, reinizializzo il calcolo
        if(self._reached):
            self._mean_counter = 0
            self._reached = False
            return "Arrivato"
            
        return self.get_hand_mean()
        
    def calculate_number(self):

        number = 0

        wirst = self._tracker.get_lm_coords(0)

        # l'indice è aperto se la distanza tra la punta e il polso è maggiore della distanza tra la nocca e il polso
        index_tip = self._tracker.get_lm_coords(8)
        index_knuckle = self._tracker.get_lm_coords(6)
        if MathUtils.distance(wirst, index_tip) > MathUtils.distance(wirst, index_knuckle):
            number += 1
            
        # verifico se il medio è aperto
        middle_tip = self._tracker.get_lm_coords(12)
        middle_knucle = self._tracker.get_lm_coords(10)
        if MathUtils.distance(wirst, middle_tip) > MathUtils.distance(wirst, middle_knucle):
            number += 1
            
        # verifico se l'anulare è aperto
        ring_tip = self._tracker.get_lm_coords(16)
        ring_knucle = self._tracker.get_lm_coords(14)
        if MathUtils.distance(wirst, ring_tip) > MathUtils.distance(wirst, ring_knucle):
            number += 1
            
        # verifico se il mignolo è aperto
        pinky_tip = self._tracker.get_lm_coords(20)
        pinky_knucle = self._tracker.get_lm_coords(18)
        if MathUtils.distance(wirst, pinky_tip) > MathUtils.distance(wirst, pinky_knucle):
            number += 1
            
        # il pollice è aperto se la distanza tra la punta e il landmark 17 è maggiore della distanza tra il landmark 17 e il polso più un certo margine
        # (quest'ultima usata come unità di misura anche se non rappresenta esattamente la distanza del pollice dal centro)
        thumb_tip = self._tracker.get_lm_coords(4)
        lm_17 = self._tracker.get_lm_coords(17)
        if MathUtils.distance(thumb_tip, lm_17) > MathUtils.distance(wirst, lm_17) * 1.1:
            number += 1
            
        return number

class ImageUtils:
    @staticmethod
    def write_on_image(image, string, pos, orientation, mode: Mode):
        if isinstance(string, Command):
            string = string.name.lower()
        pos_string = f"Position: {round(pos[0],3)}, {round(pos[1],3)}"
        orient_string = f"Orientation: {round(orientation, 3)} deg"
        # Scrive la stringa sull'immagine
        cv2.putText(image, string, (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
        cv2.putText(image, mode.name, (image.shape[1] - 150, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
        
        cv2.rectangle(image, (48, image.shape[0] - 58), (320, image.shape[0] - 15), (255,255,255), -1)
        
        cv2.putText(image, pos_string, (50, image.shape[0] - 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        cv2.putText(image, orient_string, (50, image.shape[0] - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
    @staticmethod
    def capture_image(device: cv2.VideoCapture, gesture_controller: GestureController, is_from_phone: bool):
        _, image = device.read()

        # Ho bisogno di sapere se l'immagine viene da DroidCAM perché in tal caso la devo ribaltare
        if(is_from_phone):
           image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE) 
        # Riflette l'immagine a specchio
        image = cv2.flip(image, 1)  # 1 indica il riflesso orizzontale
        
        # Rileva e disegna le mani sull'immagine
        image = gesture_controller._tracker.handsFinder(image)
        
        height, width, _ = image.shape
        new_height = int(Constants.WINDOW_WIDTH * height / width)
        image = cv2.resize(image, (Constants.WINDOW_WIDTH, new_height))
        
        gesture_controller._tracker.compute_landmarks(image)
                 
        gesture_controller._image = image
        
    @staticmethod
    def show_image(image):
         # Mostra l'immagine risultante con le mani e i landmark rilevati
        
        cv2.imshow("Video", image)

class MqttManager:
    _gesture_controller: GestureController
    _client: mqtt.Client

    def __init__(self, gesture_controller: GestureController, username="publisher", password="publisher", broker="localhost", port=1883):
        self._gesture_controller = gesture_controller

        self._client = mqtt.Client(username)
        self._client.username_pw_set(username, password, )
        self._client.on_connect = self.on_connect
        self._client.on_message = self.on_message
        
        self._counter = 0

        while True:
            try:
                result = self._client.connect(broker, port)
                if result == 0:  # Connessione riuscita
                    print("Connessione riuscita! Apertura della fotocamera..." +
                          str(self._client.is_connected()))
                    self._client.loop_start()
                    break  # Esci dal ciclo while se la connessione ha successo
                else:
                    print(
                        f"Tentativo di connessione fallito. Codice di risultato: {result}")
            except Exception as e:
                print(f"Connessione fallita, attendere l'avvio dei container")

            # Aggiungi un ritardo prima del successivo tentativo di connessione
            time.sleep(5)  # Puoi personalizzare il ritardo a tua discrezione

        self._client.publish(
            "/mode", self._gesture_controller._current_mode.name)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc), flush=True)
        self._client.subscribe("/gesture_confirm")
        self._client.subscribe("/position")

    def on_message(self, client, message, msg):
        topic = msg.topic
        decoded_msg = msg.payload.decode()
        if (topic == "/gesture_confirm"):
            print("Arrivati", flush=True)
            self._gesture_controller._reached = True
            
        if(topic == "/position"):
            pos_orient_dict = loads(decoded_msg)
            self._gesture_controller._pos = pos_orient_dict['position']
            self._gesture_controller._orient = pos_orient_dict['orientation']
        
    def publish_operation(self, current_operation):
        # Se sono in modalità automatica e l'operazione è diversa da Command.STOP, allora la ripeto all'infinito
        if(self._gesture_controller._current_mode == Mode.AUTO):
            if(current_operation == Command.STOP):
                # Se l'operazione corrente è STOP e anche l'ultima inviata è STOP, allora non invio nulla
                if(self._gesture_controller._last_operation == Command.STOP): 
                    return
                # Se l'ultima inviata non è STOP, invio STOP e setto l'ultima inviata come STOP
                self._client.publish("/commands_auto", Command.STOP.value)
                self._gesture_controller._last_operation = Command.STOP
                return
            
            # Se arrivo qui vuol dire che l'operazione attuale è una stringa contenente il numero, quindi pubblico all'infinito
            self._gesture_controller._last_operation = current_operation
            if(self._counter == 5):
                self._client.publish("/commands_auto", current_operation)
                self._counter = 0
            self._counter += 1
            return
        
        # Se arrivo qui vuol dire che sono in modalità manuale, quindi ripeto l'operazione solo se è diversa da quella precedente
        if(self._gesture_controller._last_operation != current_operation):
            print("Current operation: " + str(current_operation))
            self._client.publish("/commands_manual", 
                                 str(current_operation.value))
            self._gesture_controller._last_operation = current_operation 

def main():

    # Inizializzazione della videocamera
    device = cv2.VideoCapture(0)
    is_from_phone = False
    
    # device = cv2.VideoCapture(1)
    # is_from_phone = True

    # Inizializzazione gesture controller
    gesture_controller = GestureController()

    while True:

        time.sleep(Constants.CAM_REFRESH_TIME)
        
        # Leggo l'immagine dalla videocamera
        ImageUtils.capture_image(device, gesture_controller, is_from_phone)       

        # Calcolo l'operazione sulle mani
        current_operation = gesture_controller.compute_operation()
        
        # Scrivo l'operazione calcolata sull'immagine e la mostro
        ImageUtils.write_on_image(gesture_controller._image, current_operation, gesture_controller._pos, gesture_controller._orient, gesture_controller._current_mode)
        ImageUtils.show_image(gesture_controller._image)

        if(current_operation == "Calcolando" or current_operation == "Arrivato"):
            current_operation = Command.STOP
            
        gesture_controller._mqtt_manager.publish_operation(current_operation)
        
        # Gestione della chiusura della finestra
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or cv2.getWindowProperty("Video", cv2.WND_PROP_VISIBLE) < 1 or not gesture_controller._mqtt_manager._client.is_connected():
        # if key == 27 or cv2.getWindowProperty("Video", cv2.WND_PROP_VISIBLE) < 1:
            # Esc (27) o chiusura finestra interrompono il ciclo
            break

    # Rilascia la risorsa della videocamera e chiude tutte le finestre
    device.release()
    cv2.destroyAllWindows()
    print("Connecting to simulator...")
    RemoteAPIClient(host="localhost").require('sim').stopSimulation()
    print("Connected to SIM and simulation stopped")


if __name__ == "__main__":
    main()
