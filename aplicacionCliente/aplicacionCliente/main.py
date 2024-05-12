import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton, QGridLayout, QTextEdit, QHBoxLayout
from PyQt5.QtGui import QIcon
import paho.mqtt.client as mqtt
from collections import deque


class App(QMainWindow):
    def __init__(self):
        super().__init__()

        print("Inicializando aplicación...")

        self.setWindowTitle('Aplicacion LEGO MQTT')
        self.setGeometry(100, 100, 800, 600)

        layout = QHBoxLayout()

        buttons_and_messages_layout = QVBoxLayout()

        self.label = QLabel('Bienvenido, por favor selecciones los pedidos a realizar (minimo dos)              ODOMETRIA ->')
        buttons_and_messages_layout.addWidget(self.label)

        # Crear matriz de botones
        self.button_grid = QGridLayout()
        self.create_button_grid()
        buttons_and_messages_layout.addLayout(self.button_grid)

        # Widget de texto para mostrar los mensajes generales
        self.general_text_edit = QTextEdit()
        self.general_text_edit.setReadOnly(True)
        buttons_and_messages_layout.addWidget(self.general_text_edit)

        layout.addLayout(buttons_and_messages_layout)

        # Widget de texto para mostrar los mensajes del topic "puesto11/odometria"
        self.odometria_text_edit = QTextEdit()
        self.odometria_text_edit.setReadOnly(True)
        layout.addWidget(self.odometria_text_edit)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        print("Inicializando cliente MQTT...")
        self.init_mqtt_client()

        # Cola para los botones pulsados
        self.button_queue = deque()
        # Variable para controlar si es el primer pedido
        self.first_order = True

    def init_mqtt_client(self):
        # Configurar cliente MQTT
        self.mqtt_client = mqtt.Client(client_id='robot-client')
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Conectar al broker MQTT
        self.mqtt_client.connect("127.0.0.1", 1883, 60)
        self.mqtt_client.loop_start()

    def create_button_grid(self):
        self.buttons = [[QPushButton() for _ in range(5)] for _ in range(7)]
        for i in range(7):
            for j in range(5):
                button = self.buttons[i][j]
                button.setFixedSize(80, 80)
                button.clicked.connect(self.button_clicked)
                button.setStyleSheet("border: 1px solid black;")
                self.button_grid.addWidget(button, i, j)

    def on_connect(self, client, userdata, flags, rc):
        self.log_general_message(f"Conectado al broker MQTT con código: {rc}")
        self.mqtt_client.subscribe("map")
        self.mqtt_client.subscribe("puesto11/recibido")
        self.mqtt_client.subscribe("puesto11/odometria")

    def on_message(self, client, userdata, msg):
        self.log_general_message(f"Mensaje recibido en el topic {msg.topic}")
        if msg.topic == "map":
            map_data = msg.payload.decode()
            self.update_map(map_data)
        elif msg.topic == "puesto11/recibido" and self.button_queue:
            # Si se recibe el mensaje "Recibido" y hay botones en la cola, enviar el siguiente botón en la cola
            next_button = self.button_queue.popleft()
            message = f"{next_button[0]},{next_button[1]}"
            self.log_general_message(f"Enviando mensaje del pedido en cola: {message}")
            self.mqtt_client.publish("puesto11/envio", message)
        elif msg.topic == "puesto11/odometria":
            # Mostrar el mensaje del topic "puesto11/odometria" en el widget de texto correspondiente
            self.log_odometria_message(msg.payload.decode())
            # Obtener las coordenadas de la odometría recibida
            odometria_data = msg.payload.decode().split(",")
            row, col = int(odometria_data[0]), int(odometria_data[1])
            # Actualizar el color de fondo del botón correspondiente a las coordenadas recibidas
            button = self.buttons[row][col]
            button.setStyleSheet("background-color: yellow;")  # Cambiar el color como desees

    def update_map(self, map_data):
        map_matrix = self.generate_map_matrix(map_data)
        for i in range(7):
            for j in range(5):
                button = self.buttons[i][j]
                image_id = int(map_matrix[i][j])
                icon = QIcon(f"id{image_id}.png")
                button.setIcon(icon)
                button.setIconSize(button.size())
                # Asociar los datos al botón (los dos dígitos de la matriz)
                button.setProperty("data", (i, j))

    def generate_map_matrix(self, map_data):
        map_matrix = []
        for i in range(0, len(map_data), 2):
            number = map_data[i:i+2]
            row_index = i // 10
            col_index = i % 10 // 2
            if row_index >= 7 or col_index >= 5:
                break
            if not map_matrix:
                map_matrix = [[number]]
            elif col_index == 0:
                map_matrix.append([number])
            else:
                map_matrix[row_index].append(number)
        return map_matrix

    def button_clicked(self):
        button = self.sender()
        if button:
            # Obtener los datos asociados al botón
            data = button.property("data")
            # Si es el primer pedido y no hay pedidos en cola, enviar el mensaje del primer botón
            if self.first_order and not self.button_queue:
                message = f"{data[0]},{data[1]}"
                self.log_general_message(f"Enviando las coordenadas de recogida del primer paquete: {message}")
                self.mqtt_client.publish("puesto11/envio", message)
                self.first_order = False
            else:
                # Agregar el botón a la cola
                self.button_queue.append(data)
                self.log_general_message(f"Agregado coordenadas recogida/entrega a la cola: {data}")

    def log_general_message(self, message):
        # Agregar el mensaje al widget de texto de mensajes generales
        self.general_text_edit.append(message)
        # Imprimir el mensaje en la consola
        print(message)

    def log_odometria_message(self, message):
        # Agregar el mensaje al widget de texto de odometría
        self.odometria_text_edit.append(message)
        # Imprimir el mensaje en la consola
        print(message)


if __name__ == '__main__':
    print("Iniciando aplicación...")
    app = QApplication(sys.argv)
    window = App()
    window.show()
    sys.exit(app.exec_())
