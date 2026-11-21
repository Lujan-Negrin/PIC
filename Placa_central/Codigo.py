# Conexiones:
#   GPIO32 sensor IR de salida de la cinta
#   GPIO33 boton ACK (a GND)
#   GPIO25 LED verde, GPIO26 LED amarillo, GPIO27 LED rojo
#   GPIO18 servo de la barrera
#==========================================================

from machine import Pin, PWM
import time

# PINES
PIN_LED_VERDE, PIN_LED_AMARILLO, PIN_LED_ROJO = 25, 26, 27
PIN_SENSOR_SALIDA = 32
PIN_BOTON_ACK = 33
PIN_BARRERA = 18

led_verde = Pin(PIN_LED_VERDE, Pin.OUT)
led_amarillo = Pin(PIN_LED_AMARILLO, Pin.OUT)
led_rojo = Pin(PIN_LED_ROJO, Pin.OUT)
pin_sensor = Pin(PIN_SENSOR_SALIDA, Pin.IN, Pin.PULL_UP)
pin_ack = Pin(PIN_BOTON_ACK, Pin.IN, Pin.PULL_UP)

barrera_pwm = PWM(Pin(PIN_BARRERA, Pin.OUT))
barrera_pwm.freq(50)

# TIEMPOS Y AJUSTES
TIEMPO_DEBOUNCE_MS = 50

# Angulos del servo de la barrera
ANGULO_BARRERA_ABIERTA = 130
ANGULO_BARRERA_CERRADA = 70


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


# BARRERA
def mover_barrera(angulo):
    angulo = max(5, min(175, angulo))
    barrera_pwm.duty_u16(int(1638 + (8192 - 1638) * angulo / 180))


def abrir_barrera():
    mover_barrera(ANGULO_BARRERA_ABIERTA)
    led_verde.value(1)
    led_amarillo.value(0)
    print("[BARRERA] abierta")


def cerrar_barrera():
    mover_barrera(ANGULO_BARRERA_CERRADA)
    led_verde.value(0)
    led_amarillo.value(1)
    print("[BARRERA] cerrada")


# ARRANQUE
print("=== ESP32 CENTRAL ===")
print("Sensor de salida: cierra la barrera. Boton ACK: la abre.")
abrir_barrera()

while True:
    sensor.actualizar()
    boton_ack.actualizar()
    if sensor.flanco:
        cerrar_barrera()
    if boton_ack.flanco:
        abrir_barrera()
    time.sleep_ms(10)
