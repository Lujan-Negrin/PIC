from machine import Pin, PWM
import network
import espnow
import time
import sys
import select
import math

# PINES DE LOS SERVOS 
PIN_BASE = 12
PIN_HOMBRO = 25
PIN_CODO = 32
PIN_MUÑECA = 27

PIN_LED_TRABAJANDO = 2     # LED que indica "el brazo esta en movimiento"

# PROTOCOLO CON LA CENTRAL 
MAC_CENTRAL = b'\xAA\xBB\xCC\xDD\xEE\x01'

PIEZA_DESCONOCIDA = 0
PIEZA_TORNILLO = 1
PIEZA_TUERCA = 2
PIEZA_ARANDELA = 3

CMD_EJECUTAR = 1
CMD_PAUSA = 2

BRAZO_OK = 1
BRAZO_OCUPADO = 2
BRAZO_ERROR = 3

NOMBRE_PIEZA = {
    PIEZA_TORNILLO: "TORNILLO",
    PIEZA_TUERCA: "TUERCA",
    PIEZA_ARANDELA: "ARANDELA",
}

# Si es True, ademas de ESP-NOW tambien se puede seguir calibrando a mano
# por USB con la app de escritorio (comandos "S1:90", "S2:45", etc).
PERMITIR_CONTROL_MANUAL = True


# SERVOS 
class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin, Pin.OUT))
        self.pwm.freq(50)
        self.angulo(90)

    def angulo(self, angulo):
        duty_0 = 1638    # 0.5 ms
        duty_180 = 8192  # 2.5 ms
        duty = int(duty_0 + (duty_180 - duty_0) * angulo / 180)
        self.pwm.duty_u16(duty)

    def off(self):
        self.pwm.deinit()


class Brazo:
    def __init__(self):
        self.base = Servo(PIN_BASE)
        self.hombro = Servo(PIN_HOMBRO)
        self.codo = Servo(PIN_CODO)
        self.muñeca = Servo(PIN_MUÑECA)

    def set_angulos(self, angulos):
        self.base.angulo(angulos[0])
        self.hombro.angulo(angulos[1])
        self.codo.angulo(angulos[2])
        self.muñeca.angulo(angulos[3])

    def apagar(self):
        self.base.off()
        self.hombro.off()
        self.codo.off()
        self.muñeca.off()


# CINEMATICA INVERSA 
class CI:
    def __init__(self, l1, l2, l3, l4):
        self.l1, self.l2, self.l3, self.l4 = l1, l2, l3, l4

    def calcular(self, punto):
        x, y, z = punto[0], punto[1], punto[2] - self.l1 + self.l4

        r = math.sqrt(x**2 + y**2)
        R = math.sqrt(z**2 + r**2)
        c = (R**2 - self.l3**2 - self.l2**2) / (2 * self.l2 * self.l3)

        tita1 = math.atan2(y, x)
        tita2 = math.atan2(z, r) + math.atan2(self.l3 * math.sqrt(1 - c**2), (self.l2 + self.l3 * c))
        tita3 = math.atan2(math.sqrt(1 - c**2), c)

        tita1_deg = math.degrees(tita1)
        tita2_deg = math.degrees(tita2)
        tita3_deg = math.degrees(tita3)
        tita4_deg = 90 + tita2_deg - tita3_deg

        return [tita1_deg, tita2_deg, tita3_deg, tita4_deg]

LARGOS = (11, 13.5, 8, 9)  # (l1, l2, l3, l4)
ci = CI(*LARGOS)

SIGNO_ANGULO = [1, 1, -1, -1]   # [base, hombro, codo, muñeca]
OFFSET_ANGULO = [90, 0, 0, 0]   # [base, hombro, codo, muñeca]


def angulos_validos(angulos_crudos):
    angulos = []
    for i in range(4):
        valor = angulos_crudos[i] * SIGNO_ANGULO[i] + OFFSET_ANGULO[i]
        if valor < 0 or valor > 180:
            print("[BRAZO] Aviso: angulo fuera de rango en servo {}: {:.1f}, se recorta".format(i + 1, valor))
        angulos.append(max(0, min(180, valor)))
    return angulos


# PUNTOS (x, y, z) POR PIEZA  
POSICION_REPOSO = [90, 90, 90, 90]

PUNTOS_PIEZA = {
    PIEZA_TORNILLO: [(5, 5, 10), (5, 5, 2), (0, -8, 10)],
    PIEZA_TUERCA: [(6, 0, 10), (6, 0, 2), (0, -8, 10)],
    PIEZA_ARANDELA: [(5, -5, 10), (5, -5, 2), (0, -8, 10)],
}

TIEMPO_ENTRE_PASOS = 0.6
led_trabajando = Pin(PIN_LED_TRABAJANDO, Pin.OUT)


def mover_pieza(brazo, tipo):
    puntos = PUNTOS_PIEZA.get(tipo)
    if puntos is None:
        print("[BRAZO] Tipo de pieza sin posiciones definidas:", tipo)
        return BRAZO_ERROR

    print("[BRAZO] Moviendo para:", NOMBRE_PIEZA.get(tipo, "?"))
    led_trabajando.value(1)
    for punto in puntos:
        angulos_crudos = ci.calcular(punto)
        angulos = angulos_validos(angulos_crudos)
        brazo.set_angulos(angulos)
        time.sleep(TIEMPO_ENTRE_PASOS)
    brazo.set_angulos(POSICION_REPOSO)
    led_trabajando.value(0)
    print("[BRAZO] Movimiento terminado")
    return BRAZO_OK


# ESP-NOW 
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
enlace = espnow.ESPNow()
enlace.active(True)
enlace.add_peer(MAC_CENTRAL)


def enviar_respuesta(estado, id_secuencia):
    msg = bytes([estado, (id_secuencia >> 8) & 0xFF, id_secuencia & 0xFF])
    try:
        enlace.send(MAC_CENTRAL, msg)
    except OSError as e:
        print("Error enviando respuesta a la central:", e)


def revisar_espnow(brazo):
    host, msg = enlace.recv(0)
    if msg is None or len(msg) != 4:
        return
    comando, tipo = msg[0], msg[1]
    id_secuencia = (msg[2] << 8) | msg[3]

    if comando == CMD_EJECUTAR:
        print("[BRAZO] Orden recibida: EJECUTAR, id={}".format(id_secuencia))
        estado = mover_pieza(brazo, tipo)
        enviar_respuesta(estado, id_secuencia)
    elif comando == CMD_PAUSA:
        print("[BRAZO] Orden recibida: PAUSA, id={}".format(id_secuencia))
        led_trabajando.value(0)


# CONTROL MANUAL POR USB 
class Serial:
    def __init__(self):
        self.spoll = select.poll()
        self.spoll.register(sys.stdin, select.POLLIN)

    def verificar(self):
        return bool(self.spoll.poll(0))

    def read(self):
        lectura = ""
        while self.spoll.poll(0):
            c = sys.stdin.read(1)
            if c == '\n':
                return lectura
            lectura += c
        return "error"


SERVO_POR_INDICE = None  


def revisar_manual(serial):
    if not serial.verificar():
        return
    lectura = serial.read()
    if lectura.startswith("S") and ":" in lectura:
        try:
            indice_txt, angulo_txt = lectura[1:].split(":", 1)
            indice = int(indice_txt)
            angulo = int(angulo_txt)
        except ValueError:
            print("[MANUAL] Comando invalido:", lectura)
            return
        servo = SERVO_POR_INDICE.get(indice)
        if servo:
            servo.angulo(angulo)
        else:
            print("[MANUAL] Indice de servo desconocido:", indice)


# SETUP / LOOP 
print("ESP32 BRAZO")
print("Mi MAC:", wifi.config('mac'))
print("(copiar esta MAC en MAC_BRAZO del codigo de la central)")

brazo = Brazo()
brazo.set_angulos(POSICION_REPOSO)

SERVO_POR_INDICE = {
    1: brazo.base,
    2: brazo.hombro,
    3: brazo.codo,
    4: brazo.muñeca,
}

serial = Serial() if PERMITIR_CONTROL_MANUAL else None

print("Esperando ordenes de la central por ESP-NOW...")
if PERMITIR_CONTROL_MANUAL:
    print("(tambien acepta comandos manuales S1:90, S2:45, etc, por USB)")

while True:
    revisar_espnow(brazo)
    if PERMITIR_CONTROL_MANUAL:
        revisar_manual(serial)
    time.sleep_ms(20)