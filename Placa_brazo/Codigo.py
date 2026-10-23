from machine import Pin, PWM
import math

# PINES DE LOS SERVOS 
PIN_BASE = 12
PIN_HOMBRO = 25
PIN_CODO = 32
PIN_MUÑECA = 27

PIN_LED_TRABAJANDO = 2     # LED que indica "el brazo esta en movimiento"

PIEZA_DESCONOCIDA = 0
PIEZA_TORNILLO = 1
PIEZA_TUERCA = 2
PIEZA_ARANDELA = 3

NOMBRE_PIEZA = {
    PIEZA_TORNILLO: "TORNILLO",
    PIEZA_TUERCA: "TUERCA",
    PIEZA_ARANDELA: "ARANDELA",
}

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