# Conexiones:
#   GPIO12 servo base, GPIO25 servo hombro, GPIO32 servo codo, GPIO27 servo muñeca
#   GPIO2 LED que indica que el brazo se esta moviendo
#==========================================================

from machine import Pin, PWM
import time
import sys
import select
import math

# Pines de los servos
PIN_BASE = 12
PIN_HOMBRO = 25
PIN_CODO = 32
PIN_MUÑECA = 27

# LED: el brazo esta en movimiento
PIN_LED_TRABAJANDO = 2

# Comandos manuales por USB: S1/S2/S3/S4:<angulo>, P:x,y,z, REPOSO
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


# Cinematica inversa


class CI:
    def __init__(self, l1, l2, l3, l4):
        self.l1, self.l2, self.l3, self.l4 = l1, l2, l3, l4

    def calcular(self, punto):
        x, y, z = punto[0], punto[1], punto[2] - self.l1 + self.l4

        r = math.sqrt(x**2 + y**2)
        R = math.sqrt(z**2 + r**2)
        c = (R**2 - self.l3**2 - self.l2**2) / (2 * self.l2 * self.l3)

        if abs(c) > 1:
            return None

        tita1 = math.atan2(y, x)
        tita2 = math.atan2(z, r) + math.atan2(self.l3 * math.sqrt(1 - c**2), (self.l2 + self.l3 * c))
        tita3 = math.atan2(math.sqrt(1 - c**2), c)

        tita1_deg = math.degrees(tita1)
        tita2_deg = math.degrees(tita2)
        tita3_deg = math.degrees(tita3)

        # la muñeca compensa la inclinacion del antebrazo para que el electroiman mire hacia abajo
        tita4_deg = 90 - (tita2_deg - tita3_deg)

        return [tita1_deg, tita2_deg, tita3_deg, tita4_deg]

LARGOS = (11, 13.5, 8, 9)
ci = CI(*LARGOS)

# Conversion: angulo_servo = angulo_ci * signo + offset
SIGNO_ANGULO = [1, 1, 1, 1]
OFFSET_ANGULO = [90, 0, 0, 0]


def angulos_validos(angulos_crudos):
    angulos = []
    for i in range(4):
        valor = angulos_crudos[i] * SIGNO_ANGULO[i] + OFFSET_ANGULO[i]
        if valor < 0 or valor > 180:
            print("[BRAZO] Aviso: angulo fuera de rango en servo {}: {:.1f}, se recorta".format(i + 1, valor))
        angulos.append(max(0, min(180, valor)))
    return angulos


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


def ir_a_punto(texto):
    try:
        x, y, z = [float(v) for v in texto.split(":", 1)[1].split(",")]
    except (ValueError, IndexError):
        print("[MANUAL] Formato invalido, usar P:x,y,z  (ej. P:10,0,8)")
        return
    crudos = ci.calcular((x, y, z))
    if crudos is None:
        print("[MANUAL] Punto fuera de alcance del brazo:", (x, y, z))
        return
    print("[MANUAL] CI (base, hombro, codo, muñeca):", [round(v, 1) for v in crudos])
    angulos = angulos_validos(crudos)
    print("[MANUAL] Angulos a los servos:", [round(v, 1) for v in angulos])
    brazo.mover_suave(angulos)


def revisar_manual(serial):
    if not serial.verificar():
        return
    lectura = serial.read().strip()
    if lectura.upper() == "REPOSO":
        brazo.mover_suave(POSICION_REPOSO)
        print("[MANUAL] Brazo en reposo")
        return
    if lectura.upper().startswith("P:"):
        ir_a_punto(lectura)
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
    print("(comandos manuales por USB: S1:90, S2:45, P:10,0,8, REPOSO)")

while True:
    if PERMITIR_CONTROL_MANUAL:
        revisar_manual(serial)
    time.sleep_ms(20)
