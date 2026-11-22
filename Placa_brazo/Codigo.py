# Conexiones:
#   GPIO12 servo base, GPIO25 servo hombro, GPIO32 servo codo, GPIO27 servo muñeca
#   GPIO2 LED que indica que el brazo se esta moviendo
#   GPIO15 electroiman (a traves de un MOSFET)
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

# Electroiman (a traves de un MOSFET)
PIN_IMAN = 15
IMAN_ACTIVO_ALTO = True
TIEMPO_MAX_IMAN_MS = 60000

PIEZA_DESCONOCIDA = 0
PIEZA_TORNILLO = 1
PIEZA_TUERCA = 2
PIEZA_ARANDELA = 3

BRAZO_OK = 1
BRAZO_OCUPADO = 2
BRAZO_ERROR = 3

NOMBRE_PIEZA = {
    PIEZA_TORNILLO: "TORNILLO",
    PIEZA_TUERCA: "TUERCA",
    PIEZA_ARANDELA: "ARANDELA",
}

# Comandos manuales por USB: S1/S2/S3/S4:<angulo>, P:x,y,z, REPOSO, IMAN:1/0, PIEZA:1/2/3
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

# Punto de agarre (x, y, z) de cada pieza
ORIGEN_PIEZA = {
    PIEZA_TORNILLO: (0, 14, 2),
    PIEZA_TUERCA: (4, 14, 2),
    PIEZA_ARANDELA: (8, 14, 2),
}

# Punto de soltado sobre el contenedor 
DESTINO = (12, 0)

# Correccion de la muñeca solo en el destino, para que el electroiman quede recto
MUNECA_EXTRA_DESTINO = 17

# Alturas z (cm): Z_SOLTAR para soltar sobre el contenedor, Z_SEGURA para desplazarse sin chocar
Z_SOLTAR = 16
Z_SEGURA = 12

# Pausas (s) despues de cada movimiento
ESPERA_MOVER = 0.5
ESPERA_IMAN = 0.8
ESPERA_AGARRE = 1.0
Z_PREIMAN = 2

Z_PREIMAN_PIEZA = {}
ESPERA_AGARRE_PIEZA = {}

# Sacudida de la muneca al soltar (0 = desactivada)
SACUDIDAS_AL_SOLTAR = 0
SACUDIDA_GRADOS = 25
SACUDIDA_PAUSA_S = 0.12


def ir_a(brazo, x, y, z, espera, extra_muneca=0):
    angulos_crudos = ci.calcular((x, y, z))
    if angulos_crudos is None:
        print("[BRAZO] Punto fuera de alcance:", (x, y, z))
        return False
    angulos = angulos_validos(angulos_crudos)
    if extra_muneca:
        angulos[3] = max(ANGULO_MIN_SERVO, min(ANGULO_MAX_SERVO, angulos[3] + extra_muneca))
    brazo.mover_suave(angulos)
    time.sleep(espera)
    return True


def sacudir_muñeca(brazo):
    centro = brazo.actual[3]
    for k in range(SACUDIDAS_AL_SOLTAR):
        signo = 1 if k % 2 == 0 else -1
        brazo.muñeca.angulo(centro + signo * SACUDIDA_GRADOS)
        time.sleep(SACUDIDA_PAUSA_S)
    brazo.muñeca.angulo(centro)
    time.sleep(SACUDIDA_PAUSA_S)


def mover_pieza(brazo, tipo):
    origen = ORIGEN_PIEZA.get(tipo)
    if origen is None:
        print("[BRAZO] Tipo de pieza sin posiciones definidas:", tipo)
        return BRAZO_ERROR

    ox, oy, z_agarre = origen
    z_preiman = Z_PREIMAN_PIEZA.get(tipo, Z_PREIMAN)
    espera_agarre = ESPERA_AGARRE_PIEZA.get(tipo, ESPERA_AGARRE)
    dx, dy = DESTINO
    print("[BRAZO] Moviendo para:", NOMBRE_PIEZA.get(tipo, "?"))
    led_trabajando.value(1)

    # Secuencia: pieza -> destino -> reposo
    pasos = [
        (ox, oy, Z_SEGURA),
        (ox, oy, z_agarre + z_preiman),
        "IMAN_ON",
        (ox, oy, z_agarre),
        "ASENTAR",
        (ox, oy, Z_SEGURA),
        (dx, dy, Z_SOLTAR),
        "IMAN_OFF",
        "SACUDIR",
        (dx, dy, Z_SOLTAR + 1),
    ]
    ok = True
    for paso in pasos:
        if paso == "IMAN_ON":
            iman(True)
            time.sleep(ESPERA_IMAN)
        elif paso == "ASENTAR":
            time.sleep(espera_agarre)
        elif paso == "IMAN_OFF":
            iman(False)
            time.sleep(ESPERA_IMAN)
        elif paso == "SACUDIR":
            sacudir_muñeca(brazo)
        else:
            extra = MUNECA_EXTRA_DESTINO if (paso[0], paso[1]) == (dx, dy) else 0
            if not ir_a(brazo, paso[0], paso[1], paso[2], ESPERA_MOVER, extra):
                ok = False
                break

    iman(False)
    brazo.mover_suave(POSICION_REPOSO)
    led_trabajando.value(0)
    if ok:
        print("[BRAZO] Movimiento terminado")
        return BRAZO_OK
    return BRAZO_ERROR


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
    if lectura.upper() in ("IMAN:1", "IMAN:0"):
        iman(lectura.endswith("1"))
        return
    if lectura.upper() in ("PIEZA:1", "PIEZA:2", "PIEZA:3"):
        mover_pieza(brazo, int(lectura[-1]))
        return
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

iman_pin = None
iman_encendido = False
iman_t_on = 0


def iman(encender):
    global iman_encendido, iman_t_on
    if iman_pin is None:
        print("[IMAN] Falta definir PIN_IMAN arriba en el codigo")
        return
    iman_pin.value(1 if (encender == IMAN_ACTIVO_ALTO) else 0)
    iman_encendido = encender
    if encender:
        iman_t_on = time.ticks_ms()
    print("[IMAN] {}".format("ENCENDIDO" if encender else "apagado"))


def revisar_timeout_iman():
    if iman_encendido and time.ticks_diff(time.ticks_ms(), iman_t_on) > TIEMPO_MAX_IMAN_MS:
        iman(False)
        print("[IMAN] Apagado por seguridad (paso el tiempo maximo)")

if PIN_IMAN is not None:
    iman_pin = Pin(PIN_IMAN, Pin.OUT)
    iman(False)

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
    print("(tambien acepta comandos manuales S1:90, S2:45, P:10,0,8, REPOSO, IMAN:1, IMAN:0, por USB)")

while True:
    revisar_timeout_iman()
    if PERMITIR_CONTROL_MANUAL:
        revisar_manual(serial)
    time.sleep_ms(20)
