from machine import Pin, PWM
import sys
import select
import time

# PINES
PIN_LED_VERDE, PIN_LED_AMARILLO, PIN_LED_ROJO = 25, 26, 27
PIN_CINTA_STOP = 13  # encendido = cinta detenida
PIN_BUZZER = 14
PIN_SENSOR, PIN_BOTON_ACK = 32, 33

led_verde = Pin(PIN_LED_VERDE, Pin.OUT)
led_amarillo = Pin(PIN_LED_AMARILLO, Pin.OUT)
led_rojo = Pin(PIN_LED_ROJO, Pin.OUT)
led_cinta_stop = Pin(PIN_CINTA_STOP, Pin.OUT)
pin_sensor = Pin(PIN_SENSOR, Pin.IN, Pin.PULL_UP)
pin_ack = Pin(PIN_BOTON_ACK, Pin.IN, Pin.PULL_UP)

buzzer_pwm = PWM(Pin(PIN_BUZZER, Pin.OUT))
buzzer_pwm.freq(2000)
buzzer_pwm.duty_u16(0)

SIM_TIEMPO_BRAZO_MS = 4000

# TIPOS / COMANDOS / RESPUESTAS (deben coincidir con el brazo) 
PIEZA_DESCONOCIDA, PIEZA_TORNILLO, PIEZA_TUERCA, PIEZA_ARANDELA = 0, 1, 2, 3
CMD_EJECUTAR, CMD_PAUSA = 1, 2
BRAZO_OK, BRAZO_ERROR = 1, 3

NOMBRE_TIPO = {PIEZA_TORNILLO: "TORNILLO", PIEZA_TUERCA: "TUERCA", PIEZA_ARANDELA: "ARANDELA"}


def tipo_desde_texto(texto):
    return {"TORNILLO": PIEZA_TORNILLO, "TUERCA": PIEZA_TUERCA, "ARANDELA": PIEZA_ARANDELA}.get(texto, PIEZA_DESCONOCIDA)


# 3 ESTADOS 
EST_LIBRE, EST_PROCESANDO, EST_ERROR = range(3)

estado = EST_LIBRE
id_secuencia = 0
sim_brazo_activo = False
sim_t_inicio = 0


# ENTRADAS CON ANTIRREBOTE 
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
        if time.ticks_diff(time.ticks_ms(), self.t_cambio) > 50 and lectura != self.activa:
            self.activa = lectura
            if self.activa:
                self.flanco = True


sensor = Entrada(pin_sensor)
boton_ack = Entrada(pin_ack)


def detener_cinta(detener):
    led_cinta_stop.value(1 if detener else 0)


def cambiar_estado(nuevo):
    global estado
    estado = nuevo
    led_verde.value(1 if nuevo == EST_LIBRE else 0)
    led_amarillo.value(1 if nuevo == EST_PROCESANDO else 0)
    led_rojo.value(1 if nuevo == EST_ERROR else 0)
    buzzer_pwm.duty_u16(32768 if nuevo == EST_ERROR else 0)
    detener_cinta(nuevo == EST_ERROR)
    print("[ESTADO]", nuevo)


def enviar_orden_brazo(tipo):
    global sim_brazo_activo, id_secuencia, sim_t_inicio
    id_secuencia += 1
    print("[SIM] Orden al brazo: EJECUTAR, pieza={}, id={}".format(NOMBRE_TIPO.get(tipo, "DESCONOCIDA"), id_secuencia))
    sim_brazo_activo = True
    sim_t_inicio = time.ticks_ms()


def procesar_comando(cmd):
    cmd = cmd.strip().upper()
    if cmd == "" or estado != EST_PROCESANDO:
        return
    tipo = tipo_desde_texto(cmd)
    if tipo == PIEZA_DESCONOCIDA:
        cambiar_estado(EST_ERROR)
    else:
        enviar_orden_brazo(tipo)


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


def simular_brazo():
    global sim_brazo_activo
    if sim_brazo_activo and time.ticks_diff(time.ticks_ms(), sim_t_inicio) > SIM_TIEMPO_BRAZO_MS:
        sim_brazo_activo = False
        print("[SIM] El brazo responde: OK")
        cambiar_estado(EST_LIBRE)


print("=== ESP32 CENTRAL (version base) ===")
print("1) Activa SENSOR. 2) Escribi TORNILLO / TUERCA / ARANDELA")

lector = LectorSerial()
cambiar_estado(EST_LIBRE)

while True:
    sensor.actualizar()
    boton_ack.actualizar()
    lector.procesar()
    simular_brazo()

    if estado == EST_LIBRE and sensor.activa:
        cambiar_estado(EST_PROCESANDO)
    elif estado == EST_ERROR and boton_ack.flanco:
        cambiar_estado(EST_LIBRE)

    time.sleep_ms(10)

