from machine import Pin, PWM
import sys
import select
import time

# PINES 
PIN_LED_VERDE, PIN_LED_AMARILLO, PIN_LED_ROJO = 25, 26, 27
PIN_CINTA_STOP = 13   # salida al rele/control de la cinta
PIN_BUZZER = 14
PIN_SENSOR_SALIDA = 32
PIN_BOTON_ACK = 33
PIN_BARRERA = 18      # servo de la barrera (cambiar si se usa otro pin)

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
TIMEOUT_COLOR_MS = 5000          # espera del color despues de que el contenedor llega al sensor de salida
COLOR_VIGENCIA_MS = 30000        # un color recibido vale este tiempo
TIMEOUT_BRAZO_MS = 40000         # la secuencia del brazo (lenta) dura unos 15 a 20 s
RETARDO_CIERRE_BARRERA_MS = 0    # demora entre que el sensor detecta y la barrera baja
TIEMPO_ASENTAR_MS = 800          # espera con la barrera cerrada antes de mandar al brazo
TIEMPO_LIBERAR_MS = 1500         # con el sensor libre, cuanto espera antes de volver a LIBRE
CINTA_SE_DETIENE_AL_FRENAR = False  # True = ademas de la barrera, corta la cinta

SIM_TIEMPO_BRAZO_MS = 5000       # cuanto tarda el brazo simulado (el real tarda 15 a 20 s)

# Barrera: angulos del servo (calibrar con los comandos B:90, B:20, etc.)
ANGULO_BARRERA_ABIERTA = 0
ANGULO_BARRERA_CERRADA = 180

# TIPOS / COMANDOS / RESPUESTAS (iguales en el brazo y la camara)
PIEZA_DESCONOCIDA, PIEZA_TORNILLO, PIEZA_TUERCA, PIEZA_ARANDELA = 0, 1, 2, 3
CMD_EJECUTAR, CMD_PAUSA = 1, 2
BRAZO_OK, BRAZO_OCUPADO, BRAZO_ERROR = 1, 2, 3

NOMBRE_TIPO = {PIEZA_TORNILLO: "TORNILLO (gris)", PIEZA_TUERCA: "TUERCA (naranja)", PIEZA_ARANDELA: "ARANDELA (negro)"}


def tipo_desde_texto(texto):
    return {"TORNILLO": PIEZA_TORNILLO, "TUERCA": PIEZA_TUERCA, "ARANDELA": PIEZA_ARANDELA}.get(texto, PIEZA_DESCONOCIDA)


# ESTADOS
EST_LIBRE, EST_ESPERANDO_COLOR, EST_BRAZO_TRABAJANDO, EST_LIBERAR_CONTENEDOR, EST_ERROR = range(5)

NOMBRE_ESTADO = {
    EST_LIBRE: "LIBRE", EST_ESPERANDO_COLOR: "ESPERANDO_COLOR", EST_BRAZO_TRABAJANDO: "BRAZO_TRABAJANDO",
    EST_LIBERAR_CONTENEDOR: "LIBERAR_CONTENEDOR", EST_ERROR: "ERROR",
}

# El valor de cada causa es la cantidad de pitidos del buzzer
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


# BRAZO SIMULADO 
sim_brazo_activo = False
sim_falla = False
sim_colgar = False
sim_t_inicio = 0


def enviar_orden_brazo(comando, tipo):
    global sim_brazo_activo, sim_colgar, sim_t_inicio
    print("[SIM] Orden al brazo: {}, pieza={}, id={}".format(
        "EJECUTAR" if comando == CMD_EJECUTAR else "PAUSA", NOMBRE_TIPO.get(tipo, "DESCONOCIDA"), id_secuencia))
    if comando == CMD_EJECUTAR:
        sim_brazo_activo = not sim_colgar
        sim_colgar = False
        sim_t_inicio = time.ticks_ms()
    else:
        sim_brazo_activo = False


def guardar_color(tipo, origen):
    # el color solo se acepta mientras se espera un contenedor; despues se ignora
    global hay_color, color_tipo, t_color
    if estado not in (EST_LIBRE, EST_ESPERANDO_COLOR):
        return
    color_tipo = tipo
    t_color = time.ticks_ms()
    hay_color = True
    print("[COLOR] {} -> {}".format(origen, NOMBRE_TIPO.get(tipo, "DESCONOCIDO")))


def simular_brazo():
    global sim_brazo_activo, sim_falla, hay_respuesta, resp_estado, resp_id
    if sim_brazo_activo and time.ticks_diff(time.ticks_ms(), sim_t_inicio) > SIM_TIEMPO_BRAZO_MS:
        sim_brazo_activo = False
        resp_estado = BRAZO_ERROR if sim_falla else BRAZO_OK
        resp_id = id_secuencia
        hay_respuesta = True
        sim_falla = False
        print("[SIM] Respuesta del brazo: estado={}, id={}".format(resp_estado, resp_id))


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


# ENTRADA MANUAL POR USB (para probar sin camara ni sensor)
# TORNILLO / TUERCA / ARANDELA : simula el color que mandaria la camara
# FALLA / COLGAR               : el brazo simulado falla / no responde la proxima vez
# SENSOR                       : simula que el contenedor llego al sensor de salida
# B:<angulo>                   : mueve la barrera (para calibrar), ej. B:90
# ABRIR / CERRAR               : abre o cierra la barrera
def procesar_comando(cmd):
    global sensor_simulado, sim_falla, sim_colgar
    cmd = cmd.strip().upper()
    if cmd == "":
        return
    if cmd in ("TORNILLO", "TUERCA", "ARANDELA"):
        guardar_color(tipo_desde_texto(cmd), "manual")
    elif cmd == "SENSOR":
        sensor_simulado = True
        print("[MANUAL] sensor de salida simulado")
    elif cmd == "FALLA":
        sim_falla = True
        print("[SIM] el proximo movimiento del brazo va a fallar")
    elif cmd == "COLGAR":
        sim_colgar = True
        print("[SIM] el brazo no va a responder la proxima vez")
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
        # cualquier otra palabra se toma como un color que no es tornillo, tuerca ni arandela
        guardar_color(PIEZA_DESCONOCIDA, "manual: '{}'".format(cmd))


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

print(" ESP32 CENTRAL (SIMULACION, LOGICA POR COLOR)")
print("1) Escribi TORNILLO / TUERCA / ARANDELA (color que mandaria la camara).")
print("2) Activa el boton del SENSOR (contenedor llego al final). Cualquier otra palabra = color desconocido (error 2). Extras: FALLA, COLGAR, ABRIR, CERRAR, B:90")

lector = LectorSerial()
cambiar_estado(EST_LIBRE)

while True:
    sensor.actualizar()
    boton_ack.actualizar()
    lector.procesar()
    simular_brazo()
    actualizar_estado()
    actualizar_barrera()
    actualizar_buzzer()
    time.sleep_ms(10)