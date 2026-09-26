// ESP32 central del túnel - Proyecto Integrador (estación DLMPS-800A)
// Version inicial: estructura base y máquina de estados

#include <Arduino.h>

// ===================== MODO =====================
#define MODO_SIMULACION 1   // 1 = Wokwi, 0 = hardware real

// ===================== PINES =====================
const uint8_t PIN_LED_VERDE    = 25;
const uint8_t PIN_LED_AMARILLO = 26;
const uint8_t PIN_LED_ROJO     = 27;
const uint8_t PIN_CINTA_STOP   = 13;
const uint8_t PIN_SENSOR       = 32;

// ===================== TIEMPOS (ms) =====================
const uint32_t TIEMPO_DEBOUNCE_MS  = 50;
const uint32_t SIM_TIEMPO_BRAZO_MS = 4000;

// ===================== TIPOS =====================

enum TipoPieza : uint8_t {
  PIEZA_DESCONOCIDA = 0,
  PIEZA_TORNILLO    = 1,
  PIEZA_TUERCA      = 2,
  PIEZA_ARANDELA    = 3
};

enum Estado : uint8_t {
  EST_LIBRE,
  EST_LEYENDO_QR,
  EST_BRAZO_TRABAJANDO,
  EST_LIBERAR_CONTENEDOR
};

// ===================== VARIABLES =====================

Estado   estado     = EST_LIBRE;
uint32_t tEstado    = 0;
uint8_t  tipoActual = PIEZA_DESCONOCIDA;

volatile bool    hayQR  = false;
volatile uint8_t qrTipo = PIEZA_DESCONOCIDA;

struct Entrada {
  uint8_t  pin;
  bool     activa;
  bool     lecturaPrev;
  uint32_t tCambio;
};
Entrada sensor = {PIN_SENSOR, false, false, 0};

#if MODO_SIMULACION
String   bufferSerial   = "";
bool     simBrazoActivo = false;
uint32_t simTInicio     = 0;
#endif

// ===================== UTILIDADES =====================

const char *nombreEstado(Estado e) {
  switch (e) {
    case EST_LIBRE:              return "LIBRE";
    case EST_LEYENDO_QR:         return "LEYENDO_QR";
    case EST_BRAZO_TRABAJANDO:   return "BRAZO_TRABAJANDO";
    case EST_LIBERAR_CONTENEDOR: return "LIBERAR_CONTENEDOR";
  }
  return "?";
}

const char *nombreTipo(uint8_t t) {
  switch (t) {
    case PIEZA_TORNILLO: return "TORNILLO";
    case PIEZA_TUERCA:   return "TUERCA";
    case PIEZA_ARANDELA: return "ARANDELA";
  }
  return "DESCONOCIDA";
}

uint8_t tipoDesdeTexto(const String &texto) {
  if (texto == "TORNILLO") return PIEZA_TORNILLO;
  if (texto == "TUERCA")   return PIEZA_TUERCA;
  if (texto == "ARANDELA") return PIEZA_ARANDELA;
  return PIEZA_DESCONOCIDA;
}

void actualizarEntrada(Entrada &e) {
  bool lectura = (digitalRead(e.pin) == LOW);
  if (lectura != e.lecturaPrev) {
    e.tCambio = millis();
    e.lecturaPrev = lectura;
  }
  if ((millis() - e.tCambio) > TIEMPO_DEBOUNCE_MS) {
    e.activa = lectura;
  }
}

void detenerCinta(bool detener) {
  digitalWrite(PIN_CINTA_STOP, detener ? HIGH : LOW);
}

// ===================== MÁQUINA DE ESTADOS =====================

void cambiarEstado(Estado nuevo) {
  estado  = nuevo;
  tEstado = millis();

  digitalWrite(PIN_LED_VERDE,    nuevo == EST_LIBRE);
  digitalWrite(PIN_LED_AMARILLO, nuevo != EST_LIBRE);

  Serial.printf("[ESTADO] %s\n", nombreEstado(nuevo));
}

void actualizarEstado() {
  switch (estado) {

    case EST_LIBRE:
      if (sensor.activa) {
        Serial.println("[SENSOR] Contenedor detectado");
        detenerCinta(true);
        hayQR = false;
        cambiarEstado(EST_LEYENDO_QR);
      }
      break;

    case EST_LEYENDO_QR:
      if (hayQR) {
        hayQR = false;
        tipoActual = qrTipo;
        Serial.printf("[QR] %s\n", nombreTipo(tipoActual));
        // TODO: manejar QR desconocido y timeout (proximo commit)
#if MODO_SIMULACION
        Serial.println("[SIM] El brazo se pone a trabajar...");
        simBrazoActivo = true;
        simTInicio = millis();
#endif
        cambiarEstado(EST_BRAZO_TRABAJANDO);
      }
      break;

    case EST_BRAZO_TRABAJANDO:
#if MODO_SIMULACION
      if (simBrazoActivo && (millis() - simTInicio) > SIM_TIEMPO_BRAZO_MS) {
        simBrazoActivo = false;
        Serial.println("[SIM] El brazo termino la secuencia");
        detenerCinta(false);
        cambiarEstado(EST_LIBERAR_CONTENEDOR);
      }
#endif
      // TODO: reemplazar por la respuesta real del brazo (ESP-NOW)
      break;

    case EST_LIBERAR_CONTENEDOR:
      if (!sensor.activa) {
        detenerCinta(false);
        cambiarEstado(EST_LIBRE);
      }
      break;
  }
}

// ===================== SIMULACIÓN (Wokwi) =====================
#if MODO_SIMULACION

void procesarSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        bufferSerial.trim();
        bufferSerial.toUpperCase();
        if (estado == EST_LEYENDO_QR) {
          qrTipo = tipoDesdeTexto(bufferSerial);
          hayQR  = true;
        }
        bufferSerial = "";
      }
    } else {
      bufferSerial += c;
    }
  }
}

#endif

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_LED_VERDE,    OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO,     OUTPUT);
  pinMode(PIN_CINTA_STOP,   OUTPUT);
  pinMode(PIN_SENSOR,       INPUT_PULLUP);

  detenerCinta(false);

  Serial.println("=== ESP32 CENTRAL (version inicial) ===");
  cambiarEstado(EST_LIBRE);
}

void loop() {
  actualizarEntrada(sensor);

#if MODO_SIMULACION
  procesarSerial();
#endif

  actualizarEstado();
}
