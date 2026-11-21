# Conexiones:
#   GPIO32 sensor IR de salida de la cinta
#   GPIO33 boton ACK (a GND)
#   GPIO25 LED verde, GPIO26 LED amarillo, GPIO27 LED rojo
#   GPIO18 servo de la barrera
#   GPIO14 buzzer (1 a 4 pitidos segun el error)
#   GPIO13 salida de control de la cinta (sin uso en la version final)
#==========================================================

from machine import Pin, PWM
import network
import espnow
import sys
import select
import time

# PINES 
PIN_LED_VERDE, PIN_LED_AMARILLO, PIN_LED_ROJO = 25, 26, 27
PIN_CINTA_STOP = 13
PIN_BUZZER = 14
PIN_SENSOR_SALIDA = 32
PIN_BOTON_ACK = 33
PIN_BARRERA = 18

led_verde = Pin(PIN_LED_VERDE, Pin.OUT)
led_amarillo = Pin(PIN_LED_AMARILLO, Pin.OUT)
led_rojo = Pin(PIN_LED_ROJO, Pin.OUT)
led_cinta_stop = Pin(PIN_CINTA_STOP, Pin.OUT)
pin_sensor = Pin(PIN_SENSOR_SALIDA, Pin.IN, Pin.PULL_UP)
pin_ack = Pin(PIN_BOTON_ACK, Pin.IN, Pin.PULL_UP)

buzzer_pwm = PWM(Pin(PIN_BUZZER, Pin.OUT))
buzzer_pwm.freq(2000)
buzzer_pwm.duty_u16(0)

barrera_pwm = PWM(Pin(PIN_BARRERA, Pin.OUT))
barrera_pwm.freq(50)

# TIEMPOS Y AJUSTES 
TIEMPO_DEBOUNCE_MS = 50
TIMEOUT_COLOR_MS = 5000          # espera del color tras llegar al sensor de salida
COLOR_VIGENCIA_MS = 30000        # tiempo que vale un color recibido
TIMEOUT_BRAZO_MS = 40000         # tiempo maximo de espera de la respuesta del brazo
RETARDO_CIERRE_BARRERA_MS = 0
TIEMPO_ASENTAR_MS = 800          # espera con la barrera cerrada antes de mandar al brazo
TIEMPO_LIBERAR_MS = 1500         # espera con el sensor libre antes de volver a LIBRE
CINTA_SE_DETIENE_AL_FRENAR = False

# Angulos del servo de la barrera
ANGULO_BARRERA_ABIERTA = 130
ANGULO_BARRERA_CERRADA = 70

# TIPOS / COMANDOS / RESPUESTAS (iguales en el brazo y la camara)
PIEZA_DESCONOCIDA, PIEZA_TORNILLO, PIEZA_TUERCA, PIEZA_ARANDELA = 0, 1, 2, 3
CMD_EJECUTAR, CMD_PAUSA = 1, 2
BRAZO_OK, BRAZO_OCUPADO, BRAZO_ERROR = 1, 2, 3

NOMBRE_TIPO = {PIEZA_TORNILLO: "TORNILLO (gris)", PIEZA_TUERCA: "TUERCA (naranja)", PIEZA_ARANDELA: "ARANDELA (beige)"}


def tipo_desde_texto(texto):
    return {"TORNILLO": PIEZA_TORNILLO, "TUERCA": PIEZA_TUERCA, "ARANDELA": PIEZA_ARANDELA}.get(texto, PIEZA_DESCONOCIDA)


# MACS 
MAC_BRAZO = b'\x78\x42\x1C\x68\xFA\x84'
MAC_CAM = b'\x58\x2A\xBD\x79\x3E\x90'

# ESTADOS
EST_LIBRE, EST_ESPERANDO_COLOR, EST_BRAZO_TRABAJANDO, EST_LIBERAR_CONTENEDOR, EST_ERROR = range(5)

NOMBRE_ESTADO = {
    EST_LIBRE: "LIBRE", EST_ESPERANDO_COLOR: "ESPERANDO_COLOR", EST_BRAZO_TRABAJANDO: "BRAZO_TRABAJANDO",
    EST_LIBERAR_CONTENEDOR: "LIBERAR_CONTENEDOR", EST_ERROR: "ERROR",
}

# Cada causa de error vale su cantidad de pitidos
ERR_NINGUNO, ERR_COLOR_TIMEOUT, ERR_COLOR_DESCONOCIDO, ERR_BRAZO_TIMEOUT, ERR_BRAZO_FALLA = 0, 1, 2, 3, 4

NOMBRE_CAUSA = {
    ERR_COLOR_TIMEOUT: "No llego ningun color de la camara a tiempo",
    ERR_COLOR_DESCONOCIDO: "Color desconocido",
    ERR_BRAZO_TIMEOUT: "El brazo no respondio a tiempo",
    ERR_BRAZO_FALLA: "El brazo informo una falla",
}

estado = EST_LIBRE
causa = ERR_NINGUNO
t_estado = time.ticks_ms()
id_secuencia = 0
tipo_actual = PIEZA_DESCONOCIDA
buzzer_sonando = False

hay_color = False
color_tipo = PIEZA_DESCONOCIDA
t_color = 0
hay_respuesta = False
resp_estado = 0
resp_id = 0

barrera_cerrada = False
t_barrera_cerrada = 0
cierre_pendiente = False
t_cierre = 0
sensor_simulado = False
t_sensor_libre = 0


class Entrada:
    def __init__(self, pin):
        self.pin = pin
        self.activa = False
        self.lectura_prev = False
        self.t_cambio = time.ticks_ms()
        self.flanco = False

    def actualizar(self):
        lectura = (self.pin.value() == 0)
        self.flanco = False
        if lectura != self.lectura_prev:
            self.t_cambio = time.ticks_ms()
            self.lectura_prev = lectura
        if time.ticks_diff(time.ticks_ms(), self.t_cambio) > TIEMPO_DEBOUNCE_MS and lectura != self.activa:
            self.activa = lectura
            if self.activa:
                self.flanco = True


sensor = Entrada(pin_sensor)
boton_ack = Entrada(pin_ack)


def detener_cinta(detener):
    led_cinta_stop.value(1 if detener else 0)


def buzzer_encender():
    buzzer_pwm.duty_u16(32768)


def buzzer_apagar():
    buzzer_pwm.duty_u16(0)


def actualizar_buzzer():
    # N pitidos cortos (N = causa del error) y una pausa, en bucle
    global buzzer_sonando
    debe_sonar = False
    if estado == EST_ERROR and causa != ERR_NINGUNO:
        duracion = causa * 300
        ciclo = duracion + 1200
        t = time.ticks_diff(time.ticks_ms(), t_estado) % ciclo
        debe_sonar = (t < duracion) and ((t % 300) < 150)
    if debe_sonar != buzzer_sonando:
        buzzer_sonando = debe_sonar
        buzzer_encender() if debe_sonar else buzzer_apagar()


# BARRERA 
def mover_barrera(angulo):
    angulo = max(5, min(175, angulo))
    barrera_pwm.duty_u16(int(1638 + (8192 - 1638) * angulo / 180))


def abrir_barrera():
    global barrera_cerrada, cierre_pendiente
    cierre_pendiente = False
    barrera_cerrada = False
    mover_barrera(ANGULO_BARRERA_ABIERTA)
    print("[BARRERA] abierta")


def cerrar_barrera():
    global barrera_cerrada, t_barrera_cerrada, cierre_pendiente
    cierre_pendiente = False
    barrera_cerrada = True
    t_barrera_cerrada = time.ticks_ms()
    mover_barrera(ANGULO_BARRERA_CERRADA)
    if CINTA_SE_DETIENE_AL_FRENAR:
        detener_cinta(True)
    print("[BARRERA] cerrada")


def programar_cierre():
    global cierre_pendiente, t_cierre
    cierre_pendiente = True
    t_cierre = time.ticks_ms()


def actualizar_barrera():
    if cierre_pendiente and time.ticks_diff(time.ticks_ms(), t_cierre) >= RETARDO_CIERRE_BARRERA_MS:
        cerrar_barrera()


# ESP-NOW 
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.config(channel=1)
enlace = espnow.ESPNow()
enlace.active(True)
enlace.add_peer(MAC_BRAZO)
enlace.add_peer(MAC_CAM)


def enviar_orden_brazo(comando, tipo):
    msg = bytes([comando, tipo, (id_secuencia >> 8) & 0xFF, id_secuencia & 0xFF])
    try:
        enlace.send(MAC_BRAZO, msg)
    except OSError as e:
        print("Error enviando orden al brazo:", e)
    print("[ESPNOW->BRAZO] {}, pieza={}, id={}".format(
        "EJECUTAR" if comando == CMD_EJECUTAR else "PAUSA", NOMBRE_TIPO.get(tipo, "DESCONOCIDA"), id_secuencia))


def guardar_color(tipo, origen):
    # el color solo se acepta mientras se espera un contenedor
    global hay_color, color_tipo, t_color
    if estado not in (EST_LIBRE, EST_ESPERANDO_COLOR):
        return
    color_tipo = tipo
    t_color = time.ticks_ms()
    hay_color = True
    print("[COLOR] {} -> {}".format(origen, NOMBRE_TIPO.get(tipo, "DESCONOCIDO")))


def revisar_espnow():
    global hay_respuesta, resp_estado, resp_id
    host, msg = enlace.recv(0)
    if msg is None:
        return

    if host == MAC_BRAZO and len(msg) == 3:
        resp_estado = msg[0]
        resp_id = (msg[1] << 8) | msg[2]
        hay_respuesta = True
        print("[ESPNOW<-BRAZO] estado={}, id={}".format(resp_estado, resp_id))

    elif host == MAC_CAM and len(msg) == 1:
        guardar_color(msg[0], "camara")


def color_vigente():
    return hay_color and time.ticks_diff(time.ticks_ms(), t_color) < COLOR_VIGENCIA_MS


def cambiar_estado(nuevo):
    global estado, t_estado
    estado = nuevo
    t_estado = time.ticks_ms()
    led_verde.value(1 if nuevo == EST_LIBRE else 0)
    led_rojo.value(1 if nuevo == EST_ERROR else 0)
    led_amarillo.value(1 if nuevo in (EST_ESPERANDO_COLOR, EST_BRAZO_TRABAJANDO, EST_LIBERAR_CONTENEDOR) else 0)
    print("[ESTADO]", NOMBRE_ESTADO[nuevo])


def ir_a_error(c):
    global causa
    if estado == EST_ERROR:
        return
    causa = c
    detener_cinta(True)
    enviar_orden_brazo(CMD_PAUSA, 0)
    cambiar_estado(EST_ERROR)
    print("[ERROR] {} ({} pitidos). Apretar ACK.".format(NOMBRE_CAUSA.get(c, "?"), c))


def liberar_contenedor():
    global hay_color, t_sensor_libre
    hay_color = False
    t_sensor_libre = time.ticks_ms()
    detener_cinta(False)
    abrir_barrera()
    cambiar_estado(EST_LIBERAR_CONTENEDOR)


def actualizar_estado():
    global hay_color, id_secuencia, hay_respuesta, causa, tipo_actual, sensor_simulado, t_sensor_libre

    llego = sensor.flanco or sensor_simulado
    sensor_simulado = False

    if estado == EST_LIBRE:
        if llego:
            programar_cierre()
            cambiar_estado(EST_ESPERANDO_COLOR)

    elif estado == EST_ESPERANDO_COLOR:
        listo = barrera_cerrada and time.ticks_diff(time.ticks_ms(), t_barrera_cerrada) > TIEMPO_ASENTAR_MS
        if listo and color_vigente() and color_tipo == PIEZA_DESCONOCIDA:
            hay_color = False
            ir_a_error(ERR_COLOR_DESCONOCIDO)
        elif listo and color_vigente():
            tipo_actual = color_tipo
            hay_color = False
            id_secuencia += 1
            hay_respuesta = False
            enviar_orden_brazo(CMD_EJECUTAR, tipo_actual)
            cambiar_estado(EST_BRAZO_TRABAJANDO)
        elif time.ticks_diff(time.ticks_ms(), t_estado) > TIMEOUT_COLOR_MS + RETARDO_CIERRE_BARRERA_MS + TIEMPO_ASENTAR_MS:
            ir_a_error(ERR_COLOR_TIMEOUT)

    elif estado == EST_BRAZO_TRABAJANDO:
        if hay_respuesta:
            hay_respuesta = False
            if resp_id == id_secuencia:
                if resp_estado == BRAZO_OK:
                    liberar_contenedor()
                elif resp_estado == BRAZO_ERROR:
                    ir_a_error(ERR_BRAZO_FALLA)
        elif time.ticks_diff(time.ticks_ms(), t_estado) > TIMEOUT_BRAZO_MS:
            ir_a_error(ERR_BRAZO_TIMEOUT)

    elif estado == EST_LIBERAR_CONTENEDOR:
        # vuelve a LIBRE cuando el contenedor ya paso el sensor
        if sensor.activa:
            t_sensor_libre = time.ticks_ms()
        elif time.ticks_diff(time.ticks_ms(), t_sensor_libre) > TIEMPO_LIBERAR_MS:
            cambiar_estado(EST_LIBRE)

    elif estado == EST_ERROR:
        if boton_ack.flanco:
            causa = ERR_NINGUNO
            liberar_contenedor()


# COMANDOS POR USB (pruebas sin camara ni sensor) 
# TORNILLO / TUERCA / ARANDELA / DESCONOCIDO: simula el color de la camara
# SENSOR: simula que el contenedor llego al sensor de salida
# B:<angulo>, ABRIR, CERRAR: mueven la barrera
def procesar_comando(cmd):
    global sensor_simulado
    cmd = cmd.strip().upper()
    if cmd == "":
        return
    if cmd in ("TORNILLO", "TUERCA", "ARANDELA", "DESCONOCIDO"):
        guardar_color(tipo_desde_texto(cmd), "manual")
    elif cmd == "SENSOR":
        sensor_simulado = True
        print("[MANUAL] sensor de salida simulado")
    elif cmd == "ABRIR":
        abrir_barrera()
    elif cmd == "CERRAR":
        cerrar_barrera()
    elif cmd.startswith("B:"):
        try:
            mover_barrera(float(cmd[2:]))
            print("[MANUAL] barrera en", cmd[2:])
        except ValueError:
            print("[MANUAL] formato: B:90")
    else:
        print("[MANUAL] comando desconocido:", cmd)


class LectorSerial:
    def __init__(self):
        self.spoll = select.poll()
        self.spoll.register(sys.stdin, select.POLLIN)
        self.buffer = ""

    def procesar(self):
        while self.spoll.poll(0):
            c = sys.stdin.read(1)
            if c in ("\n", "\r"):
                if self.buffer:
                    print()
                    procesar_comando(self.buffer)
                    self.buffer = ""
            else:
                sys.stdout.write(c)
                self.buffer += c


# ARRANQUE 
buzzer_apagar()
detener_cinta(False)
abrir_barrera()

print("=== ESP32 CENTRAL (COLOR) ===")
print("Mi MAC:", wifi.config('mac'))
print("Esperando el sensor de salida y los colores de la camara...")
print("(a mano por USB: TORNILLO / TUERCA / ARANDELA / DESCONOCIDO, SENSOR, B:90, ABRIR, CERRAR)")

lector = LectorSerial()
cambiar_estado(EST_LIBRE)

while True:
    sensor.actualizar()
    boton_ack.actualizar()
    lector.procesar()
    revisar_espnow()
    actualizar_estado()
    actualizar_barrera()
    actualizar_buzzer()
    time.sleep_ms(10)
