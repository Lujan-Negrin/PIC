# Conexiones:
#   GPIO12 servo base, GPIO25 servo hombro, GPIO32 servo codo, GPIO27 servo muñeca
#   GPIO2 LED que indica que el brazo se esta moviendo
#==========================================================

from machine import Pin, PWM
import time
import sys
import select

# Pines de los servos
PIN_BASE = 12
PIN_HOMBRO = 25
PIN_CODO = 32
PIN_MUÑECA = 27

# LED: el brazo esta en movimiento
PIN_LED_TRABAJANDO = 2

# Comandos manuales por USB: S1/S2/S3/S4:<angulo>, REPOSO
PERMITIR_CONTROL_MANUAL = True

# Limites de seguridad de los servos (grados)
ANGULO_MIN_SERVO = 5
ANGULO_MAX_SERVO = 175


class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin, Pin.OUT))
        self.pwm.freq(50)
        self.angulo(90)

    def angulo(self, angulo):
        angulo = max(ANGULO_MIN_SERVO, min(ANGULO_MAX_SERVO, angulo))
        duty_0 = 1638
        duty_180 = 8192
        duty = int(duty_0 + (duty_180 - duty_0) * angulo / 180)
        self.pwm.duty_u16(duty)

    def off(self):
        self.pwm.deinit()


# Velocidad de los movimientos suaves
VELOCIDAD_GRADOS_S = 30
DURACION_MINIMA_S = 0.6
PASO_MS = 20


class Brazo:
    def __init__(self):
        self.base = Servo(PIN_BASE)
        self.hombro = Servo(PIN_HOMBRO)
        self.codo = Servo(PIN_CODO)
        self.muñeca = Servo(PIN_MUÑECA)
        self.actual = [90, 90, 90, 90]

    def set_angulos(self, angulos):
        self.base.angulo(angulos[0])
        self.hombro.angulo(angulos[1])
        self.codo.angulo(angulos[2])
        self.muñeca.angulo(angulos[3])
        self.actual = [angulos[0], angulos[1], angulos[2], angulos[3]]

    def mover_suave(self, destino):
        origen = list(self.actual)
        mayor = max(abs(destino[i] - origen[i]) for i in range(4))
        duracion = max(DURACION_MINIMA_S, mayor / VELOCIDAD_GRADOS_S)
        pasos = max(1, int(duracion * 1000 / PASO_MS))
        for k in range(1, pasos + 1):
            t = k / pasos
            f = t * t * (3 - 2 * t)
            self.set_angulos([origen[i] + (destino[i] - origen[i]) * f for i in range(4)])
            time.sleep_ms(PASO_MS)

    def apagar(self):
        self.base.off()
        self.hombro.off()
        self.codo.off()
        self.muñeca.off()


# Posiciones en cm desde el eje de la base: x hacia adelante, y hacia la izquierda
POSICION_REPOSO = [90, 90, 90, 90]

# Control manual por USB

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
    lectura = serial.read().strip()
    if lectura.upper() == "REPOSO":
        brazo.mover_suave(POSICION_REPOSO)
        print("[MANUAL] Brazo en reposo")
        return

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
            brazo.actual[indice - 1] = angulo
        else:
            print("[MANUAL] Indice de servo desconocido:", indice)


# Arranque
led_trabajando = Pin(PIN_LED_TRABAJANDO, Pin.OUT)

print("ESP32 BRAZO")

brazo = Brazo()
brazo.set_angulos(POSICION_REPOSO)

SERVO_POR_INDICE = {
    1: brazo.base,
    2: brazo.hombro,
    3: brazo.codo,
    4: brazo.muñeca,
}

serial = Serial() if PERMITIR_CONTROL_MANUAL else None

if PERMITIR_CONTROL_MANUAL:
    print("(comandos manuales por USB: S1:90, S2:45, REPOSO)")

while True:
    if PERMITIR_CONTROL_MANUAL:
        revisar_manual(serial)
    time.sleep_ms(20)
