#!/usr/bin/env pybricks-micropython
import time
from umqtt.simple import MQTTClient
import utime
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.media.ev3dev import ImageFile

from collections import deque

# VARIABLES GLOBALES
posicion_actual = (0,0)
cadena = ""
end = (0.0)
end_modificado = False
ev3 = EV3Brick()


# VARIABLES MQTT
DIRECCION_BROKER = "192.168.124.99"
CLIENT_ID = "EV3"
TOPIC = "puesto11/envio"
TOPIC_LLEGADA = "puesto11/recibido"
TOPIC_ENVIO_COORDENADAS = "puesto11/odometria"
mensaje_llegada = "Lombrices"
client = MQTTClient(CLIENT_ID, DIRECCION_BROKER)

# VARIABLES ROBOT
motor_izquierdo = Motor(Port.D)
motor_derecho = Motor(Port.A)
line_sensor = ColorSensor(Port.S4)
robot = DriveBase(motor_izquierdo, motor_derecho, wheel_diameter=55.5, axle_track=104)
giroscopio = GyroSensor(Port.S1)
DRIVE_SPEED = 100
PROPORTIONAL_GAIN = 1.4
VELOCIDAD = 100  # Velocidad en mm/s
DISTANCIA_POR_CASILLA = 280

#INICIALIZACIÓN MATRIZ
num_filas = 7
num_columnas = 5
recorridoMatriz = num_columnas * num_filas

mapa_codificado = "0202000105030705000200041109060110031000000200080101100110000106010701"

matrizInsercion = []
for i in range(num_filas):
    fila = []
    for j in range(num_columnas):
        fila.append(0)
    matrizInsercion.append(fila)
lista_convertida = []

texto = ""

conversionFloat = 0
incremento = 0
for fila in range(num_filas):
    for columna in range(num_columnas):
        texto = mapa_codificado[incremento] + mapa_codificado[incremento + 1]
        conversionFloat = float(texto)
        lista_convertida.append(texto + ", ")
        matrizInsercion[fila][columna] = conversionFloat
        incremento = incremento + 2



# *****************ALGORITMO CÁLCULO CAMINO*****************
def caminoValido(matrix, start, end):
    # Direcciones posibles: arriba, abajo, izquierda, derecha
    directions = [(1, 0, "derecha"), (-1, 0, "izquierda"), (0, 1, "recto"), (0, -1, "atras")]
    n = len(matrix)
    m = len(matrix[0])  # Se asume que todas las filas tienen la misma longitud

    # Función para verificar si una casilla está dentro de la matriz y es accesible
    def is_valid(x, y):
        return 0 <= x < n and 0 <= y < m and matrix[x][y] != 0.0

    visited = set()
    queue = deque([(start, [])])  # (posición, lista de movimientos)

    while queue:
        (x, y), path = queue.popleft()
        if (x, y) == end:
            return path

        for dx, dy, move in directions:
            new_x, new_y = x + dx, y + dy
            if is_valid(new_x, new_y) and (new_x, new_y) not in visited:
                visited.add((new_x, new_y))
                queue.append(((new_x, new_y), path + [move]))

    # Si no se encuentra un camino válido
    return None
'''
def avanzar_casillas(n_casillas, angulo):
    TIEMPO_AVANCE = n_casillas * (DISTANCIA_POR_CASILLA / VELOCIDAD)
    tiempo_inicial = time.time()
    while (time.time() - tiempo_inicial < TIEMPO_AVANCE):
        robot.drive(100, angulo)
'''



# *****************MÉTODOS MOVIMIENTO ROBOT *****************

def giro_izq():
    while(giroscopio.angle() > -85):
        robot.drive(5, 15)
    giroscopio.reset_angle(0)
        #print(giroscopio.angle())

def giro_der():
    while(giroscopio.angle() < 85):
        robot.drive(5, -15)
    giroscopio.reset_angle(0)
        #print(giroscopio.angle())
    
def avanzar_casillas_atras():
    #giro_der()
    while(giroscopio.angle() < 180):
        robot.drive(5, -15)
    giroscopio.reset_angle(0)
    #giro_der()
    avanzar_casillas(1,0)

def avanzar_casillas_lateral(n_casillas, angulo):
    global posicion_actual
    if(angulo < 0):
        print("giro izq")
        giro_izq()
    else:
        giro_der()
    #time.sleep(1)
    valor_color = line_sensor.rgb()
    TIEMPO_AVANCE = n_casillas * (DISTANCIA_POR_CASILLA / VELOCIDAD)
    TIEMPO_SECUNDARIO = TIEMPO_AVANCE - (TIEMPO_AVANCE * 0.20)

    tiempo_inicial = time.time()
    while (time.time() - tiempo_inicial < TIEMPO_AVANCE and (not(es_color_negro(valor_color) and time.time() - tiempo_inicial > TIEMPO_SECUNDARIO))):
        robot.drive(100, 0)
        valor_color = line_sensor.rgb()
    posicion_actual = actualizar_posicion(posicion_actual, n_casillas, angulo)
    mandar_coordenadas()

    '''
    giroscopio.reset_angle(0)
    print("giro")
    if(angulo < 0):
        print("giro der")
        giro_der()
        
    else:
        giro_izq()
    '''

def avanzar_casillas(n_casillas, angulo):
    global posicion_actual
    TIEMPO_AVANCE = n_casillas * (DISTANCIA_POR_CASILLA / VELOCIDAD)
    TIEMPO_SECUNDARIO = TIEMPO_AVANCE - (TIEMPO_AVANCE * 0.20)
    tiempo_inicial = time.time()
    valor_color = line_sensor.rgb()
    while (time.time() - tiempo_inicial < TIEMPO_AVANCE and (not(es_color_negro(valor_color) and time.time() - tiempo_inicial > TIEMPO_SECUNDARIO))):
        robot.drive(100, 0)
        valor_color = line_sensor.rgb()
    posicion_actual = actualizar_posicion(posicion_actual, n_casillas, angulo)
    mandar_coordenadas()

def switch(movimiento):
    if movimiento == 'recto':
        avanzar_casillas(1,0)
        return "recto"
    elif movimiento == 'derecha':
        avanzar_casillas_lateral(1,85)
        return "derecha"
    elif movimiento == 'izquierda':
        avanzar_casillas_lateral(1,-85)
        return "izquierda"
    elif movimiento == 'atras':
        avanzar_casillas_atras()
        return "atras"

def switch_der(movimiento):
    if movimiento == 'recto':
        avanzar_casillas_lateral(1,-85)
        return "recto"
    elif movimiento == 'derecha':
        avanzar_casillas(1,0)
        return "derecha"
    elif movimiento == 'izquierda':
        avanzar_casillas_atras()
        return "izquierda"
    elif movimiento == 'atras':
        avanzar_casillas_lateral(1,85)
        return "atras"

def switch_izq(movimiento):
    if movimiento == 'recto':
        avanzar_casillas_lateral(1,85)
        return "recto"
    elif movimiento == 'derecha':
        avanzar_casillas_atras()
        return "derecha"
    elif movimiento == 'izquierda':
        avanzar_casillas(1,0)
        return "izquierda"
    elif movimiento == 'atras':
        avanzar_casillas_lateral(1,-85)
        return "atras"

def switch_atras(movimiento):
    if movimiento == 'recto':
        avanzar_casillas_atras()
        return "recto"
    elif movimiento == 'derecha':
        avanzar_casillas_lateral(1,-85)
        return "derecha"
    elif movimiento == 'izquierda':
        avanzar_casillas_lateral(1,85)
        return "izquierda"
    elif movimiento == 'atras':
        avanzar_casillas(1,0)
        return "atras"



#***************** MÉTODOS DE MENSAJES MQTT *****************
def recibirMensaje(topic, msg):
    global end
    global end_modificado
    text = msg.decode("utf-8")
    cadena = text
    coordenada_x = procesarCadenaX(cadena) 
    coordenada_y = procesarCadenaY(cadena)
    end = (coordenada_x, coordenada_y)
    global cont
    end_modificado = True

def procesarCadenaX(cadena):
    # Dividir la cadena en base a la coma y extraer el primer elemento
    coordenada_x = cadena.split(',')[0]
    # Convertir la coordenada x a un entero y devolverla
    print("procesarcadenaX: "+coordenada_x)
    return int(coordenada_x)

def procesarCadenaY(cadena):
    # Dividir la cadena en base a la coma y extraer el segundo elemento
    coordenada_y = cadena.split(',')[1]
    # Convertir la coordenada y a un entero y devolverla
    print("procesarcadenaY: "+coordenada_y)
    return int(coordenada_y)

# Publicamos la llegada
def mandar_mensaje_llegada():
    global end_modificado
    client.publish(TOPIC_LLEGADA, mensaje_llegada)
    end_modificado = False

def mandar_coordenadas():
    coordenadas_str = ",".join(str(coord) for coord in posicion_actual)
    client.publish(TOPIC_ENVIO_COORDENADAS, coordenadas_str)


# ***************** FUNCIONES UMBRALES COLORES *****************
# Función para determinar si el color es verde
def es_color_verde(rgb_color):
    # Define el umbral para el verde en el espacio RGB
    UMBRAL_VERDE = (20, 50, 16)  # Valores RGB que indican verde claro
    
    # Compara el color detectado con el umbral para el verde
    return all(rgb >= umbral for rgb, umbral in zip(rgb_color, UMBRAL_VERDE))

# Función para determinar si el color es negro
def es_color_negro(rgb_color):
    # Define el umbral para el negro en el espacio RGB
    UMBRAL_NEGRO = (20, 20, 20)  # Valores RGB que indican negro
    print("Negro loquete")
    
    # Compara el color detectado con el umbral para el negro
    return all(rgb <= umbral for rgb, umbral in zip(rgb_color, UMBRAL_NEGRO))

def es_color_blanco(rgb_color):
    # Define el umbral para el blanco en el espacio RGB
    UMBRAL_BLANCO = (200, 200, 200)  # Valores RGB que indican blanco
    
    # Compara el color detectado con el umbral para el blanco
    return all(rgb >= umbral for rgb, umbral in zip(rgb_color, UMBRAL_BLANCO))

def es_color_azul(rgb_color):
    # Define el umbral para el azul en el espacio RGB
    UMBRAL_AZUL = (0, 0, 50)  # Valores RGB que indican azul
    
    # Compara el color detectado con el umbral para el azul
    return all(rgb >= umbral for rgb, umbral in zip(rgb_color, UMBRAL_AZUL))

def es_color_rojo(rgb_color):
    # Define el umbral para el rojo en el espacio RGB
    UMBRAL_ROJO = (50, 0, 0)  # Valores RGB que indican rojo
    
    # Compara el color detectado con el umbral para el rojo
    #return all(rgb >= umbral for rgb, umbral in zip(rgb_color, UMBRAL_ROJO))

def actualizar_posicion(posicion, orientacion, n_casillas):
    x, y = posicion
    if orientacion == "recto":
        y += n_casillas  # Avanza hacia adelante
    elif orientacion == "derecha":
        x += n_casillas  # Avanza hacia la derecha
    elif orientacion == "izquierda":
        x -= n_casillas  # Avanza hacia la izquierda
    elif orientacion == "atras":
        y -= n_casillas  # Retrocede
    return x, y


# Inicialización MQTT
client.set_callback(recibirMensaje)
client.connect()
client.subscribe(TOPIC)
global cont
cont = 0
# ***************** BUCLE PRINCIPAL *****************
#while True:

    # Inicialización coordenadas
orientacion = "recto"
start = (6, 0)  # Posición de inicio
print(end_modificado)
print("Lolito no llega")
client.wait_msg()  # Espera hasta que llegue un mensaje
print(end)
# print("Variable de fin de recorrido actualizada sin errores")
path = caminoValido(matrizInsercion, start, end)


# Recorrido matrizs
i = 0
print(path)
for row in matrizInsercion:
    for elem in row:
        print(elem, end=', ')
    print()
print(matrizInsercion[6][0])

print("Comenzamos recogida")
giroscopio.reset_angle(0)
if path:
    for movimiento in path:
        if orientacion == "recto":
            orientacion = switch(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "derecha":
            orientacion = switch_der(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "izquierda":
            orientacion = switch_izq(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "atras":
            orientacion = switch_atras(movimiento)
            giroscopio.reset_angle(0)
else:
    print("No hay camino")
print("Terminado Fase 1")
mandar_mensaje_llegada()


start = end
print(start)
print(end_modificado)
# print("Lolito no llega")
client.wait_msg()  # Espera hasta que llegue un mensaje
print(end)
# print("Variable de fin de recorrido actualizada sin errores")
path = caminoValido(matrizInsercion, start, end)


# Recorrido matrizs
i = 0
print(path)
for row in matrizInsercion:
    for elem in row:
        print(elem, end=', ')
    print()
print(matrizInsercion[6][0])

print("Comenzamos recogida")

if path:
    for movimiento in path:
        if orientacion == "recto":
            orientacion = switch(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "derecha":
            orientacion = switch_der(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "izquierda":
            orientacion = switch_izq(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "atras":
            orientacion = switch_atras(movimiento)
            giroscopio.reset_angle(0)
else:
    print("No hay camino")
print("Terminado Fase 2")
mandar_mensaje_llegada()

start = end
print(start)
print(end_modificado)
# print("Lolito no llega")
client.wait_msg()  # Espera hasta que llegue un mensaje
print(end)
# print("Variable de fin de recorrido actualizada sin errores")
path = caminoValido(matrizInsercion, start, end)


# Recorrido matrizs
i = 0
print(path)
for row in matrizInsercion:
    for elem in row:
        print(elem, end=', ')
    print()
print(matrizInsercion[6][0])

print("Comenzamos recogida")

if path:
    for movimiento in path:
        if orientacion == "recto":
            orientacion = switch(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "derecha":
            orientacion = switch_der(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "izquierda":
            orientacion = switch_izq(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "atras":
            orientacion = switch_atras(movimiento)
            giroscopio.reset_angle(0)
else:
    print("No hay camino")
print("Terminado Fase 3")
mandar_mensaje_llegada()


start = end
print(start)
print(end_modificado)
# print("Lolito no llega")
client.wait_msg()  # Espera hasta que llegue un mensaje
print(end)
# print("Variable de fin de recorrido actualizada sin errores")
path = caminoValido(matrizInsercion, start, end)


# Recorrido matrizs
i = 0
print(path)
for row in matrizInsercion:
    for elem in row:
        print(elem, end=', ')
    print()
print(matrizInsercion[6][0])

print("Comenzamos recogida")

if path:
    for movimiento in path:
        if orientacion == "recto":
            orientacion = switch(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "derecha":
            orientacion = switch_der(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "izquierda":
            orientacion = switch_izq(movimiento)
            giroscopio.reset_angle(0)
        elif orientacion == "atras":
            orientacion = switch_atras(movimiento)
            giroscopio.reset_angle(0)
else:
    print("No hay camino")
print("Terminado Fase 1")
mandar_mensaje_llegada()
'''
start = (4, 4)  # Posición de inicio
end = (0, 0)    # Posición de destino

path = caminoValido(matrizInsercion, start, end)
i = 0
print(path)
for row in matrizInsercion:
    for elem in row:
        print(elem, end=', ')
    print()
print(matrizInsercion[6][0])

if path:
    for movimiento in path:
        giroscopio.reset_angle(0)
        switch.get(movimiento, lambda: None)()
        print(movimiento)
else:
    print("No hay camino")


giro_izq()
giroscopio.reset_angle(0)
giro_der()

#robot.turn(100)
'''