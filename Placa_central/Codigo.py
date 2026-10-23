from machine import Pin, PWM
import sys
import select
import time

# PINES
PIN_LED_VERDE, PIN_LED_AMARILLO, PIN_LED_ROJO = 25, 26, 27
PIN_CINTA_STOP = 13  # en la simulacion: encendido = cinta detenida. En el prototipo real: salida al rele de la cinta.
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

TIEMPO_DEBOUNCE_MS = 50
TIMEOUT_QR_MS = 20000
TIMEOUT_BRAZO_MS = 10000
SIM_TIEMPO_BRAZO_MS = 4000

# TIPOS / COMANDOS / RESPUESTAS (deben coincidir con el brazo)
PIEZA_DESCONOCIDA, PIEZA_TORNILLO, PIEZA_TUERCA, PIEZA_ARANDELA = 0, 1, 2, 3
CMD_EJECUTAR, CMD_PAUSA = 1, 2
BRAZO_OK, BRAZO_OCUPADO, BRAZO_ERROR = 1, 2, 3

NOMBRE_TIPO = {PIEZA_TORNILLO: "TORNILLO", PIEZA_TUERCA: "TUERCA", PIEZA_ARANDELA: "ARANDELA"}


def tipo_desde_texto(texto):
    return {"TORNILLO": PIEZA_TORNILLO, "TUERCA": PIEZA_TUERCA, "ARANDELA": PIEZA_ARANDELA}.get(texto, PIEZA_DESCONOCIDA)


# ESTADOS
EST_LIBRE, EST_LEYENDO_QR, EST_BRAZO_TRABAJANDO, EST_LIBERAR_CONTENEDOR, EST_ERROR = range(5)

NOMBRE_ESTADO = {
    EST_LIBRE: "LIBRE", EST_LEYENDO_QR: "LEYENDO_QR", EST_BRAZO_TRABAJANDO: "BRAZO_TRABAJANDO",
    EST_LIBERAR_CONTENEDOR: "LIBERAR_CONTENEDOR", EST_ERROR: "ERROR",
}

# El valor de cada causa es la cantidad de pitidos del buzzer
ERR_NINGUNO, ERR_QR_TIMEOUT, ERR_QR_DESCONOCIDO, ERR_BRAZO_TIMEOUT, ERR_BRAZO_FALLA = 0, 1, 2, 3, 4

NOMBRE_CAUSA = {
    ERR_QR_TIMEOUT: "No se leyo ningun QR a tiempo",
    ERR_QR_DESCONOCIDO: "QR desconocido",
    ERR_BRAZO_TIMEOUT: "El brazo no respondio a tiempo",
    ERR_BRAZO_FALLA: "El brazo informo una falla",
}

estado = EST_LIBRE
causa = ERR_NINGUNO
t_estado = time.ticks_ms()
id_secuencia = 0
buzzer_sonando = False

hay_qr = False
qr_tipo = PIEZA_DESCONOCIDA
hay_respuesta = False
resp_estado = 0
resp_id = 0

sim_brazo_activo = False
sim_falla = False
sim_colgar = False
sim_t_inicio = 0


class Entrada:
    def __init__(self, pin):
        self.pin = pin
        self.activa = False
        self.lectura_prev = False
        self.t_cambio = time.ticks_ms()
        self.flanco = False

    def actualizar(self):
        # con antirrebote (debounce)
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
    # patron: N pitidos cortos (N = causa del error) y una pausa, en bucle
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


def enviar_orden_brazo(comando, tipo):
    # en el hardware real esto manda el mensaje por ESP-NOW; aca se simula
    global sim_brazo_activo, sim_colgar, sim_t_inicio
    print("[SIM] Orden al brazo: {}, pieza={}, id={}".format(
        "EJECUTAR" if comando == CMD_EJECUTAR else "PAUSA", NOMBRE_TIPO.get(tipo, "DESCONOCIDA"), id_secuencia))
    if comando == CMD_EJECUTAR:
        sim_brazo_activo = not sim_colgar
        sim_colgar = False
        sim_t_inicio = time.ticks_ms()
    else:
        sim_brazo_activo = False


def cambiar_estado(nuevo):
    global estado, t_estado
    estado = nuevo
    t_estado = time.ticks_ms()
    led_verde.value(1 if nuevo == EST_LIBRE else 0)
    led_rojo.value(1 if nuevo == EST_ERROR else 0)
    led_amarillo.value(1 if nuevo in (EST_LEYENDO_QR, EST_BRAZO_TRABAJANDO, EST_LIBERAR_CONTENEDOR) else 0)
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


def actualizar_estado():
    global hay_qr, qr_tipo, tipo_actual, id_secuencia, hay_respuesta, causa

    if estado == EST_LIBRE:
        if sensor.activa:
            detener_cinta(True)
            hay_qr = False
            cambiar_estado(EST_LEYENDO_QR)

    elif estado == EST_LEYENDO_QR:
        if hay_qr:
            hay_qr = False
            tipo = qr_tipo
            if tipo == PIEZA_DESCONOCIDA:
                ir_a_error(ERR_QR_DESCONOCIDO)
            else:
                tipo_actual = tipo
                id_secuencia += 1
                hay_respuesta = False
                enviar_orden_brazo(CMD_EJECUTAR, tipo_actual)
                cambiar_estado(EST_BRAZO_TRABAJANDO)
        elif time.ticks_diff(time.ticks_ms(), t_estado) > TIMEOUT_QR_MS:
            ir_a_error(ERR_QR_TIMEOUT)

    elif estado == EST_BRAZO_TRABAJANDO:
        if hay_respuesta:
            hay_respuesta = False
            if resp_id == id_secuencia:
                if resp_estado == BRAZO_OK:
                    detener_cinta(False)
                    cambiar_estado(EST_LIBERAR_CONTENEDOR)
                elif resp_estado == BRAZO_ERROR:
                    ir_a_error(ERR_BRAZO_FALLA)
        elif time.ticks_diff(time.ticks_ms(), t_estado) > TIMEOUT_BRAZO_MS:
            ir_a_error(ERR_BRAZO_TIMEOUT)

    elif estado == EST_LIBERAR_CONTENEDOR:
        if not sensor.activa:
            detener_cinta(False)
            cambiar_estado(EST_LIBRE)

    elif estado == EST_ERROR:
        if boton_ack.flanco:
            causa = ERR_NINGUNO
            cambiar_estado(EST_LIBERAR_CONTENEDOR)


def procesar_comando(cmd):
    global sim_falla, sim_colgar, qr_tipo, hay_qr
    cmd = cmd.strip().upper()
    if cmd == "":
        return
    if cmd == "FALLA":
        sim_falla = True
    elif cmd == "COLGAR":
        sim_colgar = True
    elif estado == EST_LEYENDO_QR:
        qr_tipo = tipo_desde_texto(cmd)
        hay_qr = True


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
    global sim_brazo_activo, resp_estado, resp_id, hay_respuesta, sim_falla
    if sim_brazo_activo and time.ticks_diff(time.ticks_ms(), sim_t_inicio) > SIM_TIEMPO_BRAZO_MS:
        sim_brazo_activo = False
        resp_estado = BRAZO_ERROR if sim_falla else BRAZO_OK
        resp_id = id_secuencia
        hay_respuesta = True
        sim_falla = False


# SETUP / LOOP
buzzer_apagar()
detener_cinta(False)

print("=== ESP32 CENTRAL (SIMULACION) ===")
print("1) Activa SENSOR. 2) Escribi TORNILLO / TUERCA / ARANDELA. Extras: FALLA / COLGAR")

lector = LectorSerial()
cambiar_estado(EST_LIBRE)

while True:
    sensor.actualizar()
    boton_ack.actualizar()
    lector.procesar()
    simular_brazo()
    actualizar_estado()
    actualizar_buzzer()
    time.sleep_ms(10)

