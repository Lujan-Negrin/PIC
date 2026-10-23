// ESP32 central del túnel - Proyecto Integrador (estación DLMPS-800A)

#include <Arduino.h>

// MODO
#define MODO_SIMULACION 1   // 1 = Wokwi, 0 = hardware real
#define BUZZER_ACTIVO   0   // 1 = buzzer activo (digitalWrite), 0 = pasivo (tone)

#if !MODO_SIMULACION
  #include <WiFi.h>
  #include <esp_now.h>
  // Reemplazar por las MAC reales
  uint8_t MAC_CAM[]   = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};
  uint8_t MAC_BRAZO[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02};
#endif

// PINES
const uint8_t PIN_LED_VERDE    = 25;
const uint8_t PIN_LED_AMARILLO = 26;
const uint8_t PIN_LED_ROJO     = 27;
// En Wokwi este LED azul representa la cinta: encendido = cinta DETENIDA.
// En el equipo real, reemplazar por la salida hacia el relé/PLC de la cinta.
const uint8_t PIN_CINTA_STOP   = 13;
const uint8_t PIN_BUZZER       = 14;
const uint8_t PIN_SENSOR       = 32;
const uint8_t PIN_BOTON_ACK    = 33;

// TIEMPOS (ms)
const uint32_t TIEMPO_DEBOUNCE_MS  = 50;
#if MODO_SIMULACION
const uint32_t TIMEOUT_QR_MS       = 20000;
#else
const uint32_t TIMEOUT_QR_MS       = 5000;
#endif
const uint32_t TIMEOUT_BRAZO_MS    = 10000;
const uint32_t SIM_TIEMPO_BRAZO_MS = 4000;

// MENSAJES
// Estas estructuras deben ser IDÉNTICAS en la ESP32-CAM y en la ESP32 del brazo.

enum TipoPieza : uint8_t {
  PIEZA_DESCONOCIDA = 0,
  PIEZA_TORNILLO    = 1,
  PIEZA_TUERCA      = 2,
  PIEZA_ARANDELA    = 3
};

enum Comando : uint8_t {
  CMD_EJECUTAR = 1,
  CMD_PAUSA    = 2
};

enum EstadoBrazo : uint8_t {
  BRAZO_OK      = 1,
  BRAZO_OCUPADO = 2,
  BRAZO_ERROR   = 3
};

typedef struct __attribute__((packed)) { uint8_t tipo; } MensajeQR;
typedef struct __attribute__((packed)) { uint8_t comando; uint8_t tipo; uint16_t id; } MensajeOrden;
typedef struct __attribute__((packed)) { uint8_t estado; uint16_t id; } MensajeRespuesta;

// ESTADOS

enum Estado : uint8_t {
  EST_LIBRE,
  EST_LEYENDO_QR,
  EST_BRAZO_TRABAJANDO,
  EST_LIBERAR_CONTENEDOR,
  EST_ERROR
};

// El valor de cada causa es la cantidad de pitidos del buzzer
enum CausaError : uint8_t {
  ERR_NINGUNO        = 0,
  ERR_QR_TIMEOUT     = 1,
  ERR_QR_DESCONOCIDO = 2,
  ERR_BRAZO_TIMEOUT  = 3,
  ERR_BRAZO_FALLA    = 4
};

// VARIABLES

Estado     estado      = EST_LIBRE;
CausaError causa       = ERR_NINGUNO;
uint32_t   tEstado     = 0;
uint16_t   idSecuencia = 0;
uint8_t    tipoActual  = PIEZA_DESCONOCIDA;
bool       buzzerSonando = false;

volatile bool     hayQR        = false;
volatile uint8_t  qrTipo       = PIEZA_DESCONOCIDA;
volatile bool     hayRespuesta = false;
volatile uint8_t  respEstado   = 0;
volatile uint16_t respId       = 0;

struct Entrada {
  uint8_t  pin;
  bool     activa;
  bool     lecturaPrev;
  uint32_t tCambio;
  bool     flanco;
};
Entrada sensor   = {PIN_SENSOR,    false, false, 0, false};
Entrada botonAck = {PIN_BOTON_ACK, false, false, 0, false};

#if MODO_SIMULACION
String   bufferSerial = "";
bool     simBrazoActivo = false;
bool     simFalla   = false;
bool     simColgar  = false;
uint32_t simTInicio = 0;
#endif

// UTILIDADES

const char *nombreEstado(Estado e) {
  switch (e) {
    case EST_LIBRE:              return "LIBRE";
    case EST_LEYENDO_QR:         return "LEYENDO_QR";
    case EST_BRAZO_TRABAJANDO:   return "BRAZO_TRABAJANDO";
    case EST_LIBERAR_CONTENEDOR: return "LIBERAR_CONTENEDOR";
    case EST_ERROR:              return "ERROR";
  }
  return "?";
}

const char *nombreCausa(CausaError c) {
  switch (c) {
    case ERR_QR_TIMEOUT:     return "No se leyo ningun QR a tiempo";
    case ERR_QR_DESCONOCIDO: return "QR desconocido";
    case ERR_BRAZO_TIMEOUT:  return "El brazo no respondio a tiempo";
    case ERR_BRAZO_FALLA:    return "El brazo informo una falla";
    default:                 return "-";
  }
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
  e.flanco = false;
  if (lectura != e.lecturaPrev) {
    e.tCambio = millis();
    e.lecturaPrev = lectura;
  }
  if ((millis() - e.tCambio) > TIEMPO_DEBOUNCE_MS && lectura != e.activa) {
    e.activa = lectura;
    if (e.activa) e.flanco = true;
  }
}

void detenerCinta(bool detener) {
  digitalWrite(PIN_CINTA_STOP, detener ? HIGH : LOW);
}

void buzzerEncender() {
#if BUZZER_ACTIVO
  digitalWrite(PIN_BUZZER, HIGH);
#else
  tone(PIN_BUZZER, 2000);
#endif
}

void buzzerApagar() {
#if BUZZER_ACTIVO
  digitalWrite(PIN_BUZZER, LOW);
#else
  noTone(PIN_BUZZER);
#endif
}

void actualizarBuzzer() {
  bool debeSonar = false;
  if (estado == EST_ERROR && causa != ERR_NINGUNO) {
    uint32_t pitidos  = (uint32_t)causa;
    uint32_t duracion = pitidos * 300UL;
    uint32_t ciclo    = duracion + 1200UL;
    uint32_t t        = (millis() - tEstado) % ciclo;
    debeSonar = (t < duracion) && ((t % 300UL) < 150UL);
  }
  if (debeSonar != buzzerSonando) {
    buzzerSonando = debeSonar;
    if (debeSonar) buzzerEncender(); else buzzerApagar();
  }
}

// COMUNICACIÓN CON EL BRAZO

void enviarOrdenBrazo(uint8_t comando, uint8_t tipo) {
  MensajeOrden m;
  m.comando = comando;
  m.tipo    = tipo;
  m.id      = idSecuencia;

#if MODO_SIMULACION
  Serial.printf("[SIM] Orden al brazo: %s, pieza=%s, id=%u\n",
                comando == CMD_EJECUTAR ? "EJECUTAR" : "PAUSA", nombreTipo(tipo), idSecuencia);
  if (comando == CMD_EJECUTAR) {
    simBrazoActivo = !simColgar;
    simColgar = false;
    simTInicio = millis();
  } else {
    simBrazoActivo = false;
  }
#else
  esp_err_t r = esp_now_send(MAC_BRAZO, (uint8_t *)&m, sizeof(m));
  if (r != ESP_OK) Serial.printf("Error enviando al brazo (codigo %d)\n", r);
#endif
}

// MÁQUINA DE ESTADOS

void cambiarEstado(Estado nuevo) {
  estado  = nuevo;
  tEstado = millis();

  digitalWrite(PIN_LED_VERDE,    nuevo == EST_LIBRE);
  digitalWrite(PIN_LED_ROJO,     nuevo == EST_ERROR);
  digitalWrite(PIN_LED_AMARILLO, nuevo == EST_LEYENDO_QR ||
                                 nuevo == EST_BRAZO_TRABAJANDO ||
                                 nuevo == EST_LIBERAR_CONTENEDOR);

  Serial.printf("[ESTADO] %s\n", nombreEstado(nuevo));
}

void irAError(CausaError c) {
  if (estado == EST_ERROR) return;
  causa = c;
  detenerCinta(true);
  enviarOrdenBrazo(CMD_PAUSA, 0);
  cambiarEstado(EST_ERROR);
  Serial.printf("[ERROR] %s (%u pitidos). Apretar el boton de reconocimiento.\n", nombreCausa(c), (uint8_t)c);
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
        uint8_t tipo = qrTipo;
        Serial.printf("[QR] %s\n", nombreTipo(tipo));
        if (tipo == PIEZA_DESCONOCIDA) {
          irAError(ERR_QR_DESCONOCIDO);
        } else {
          tipoActual = tipo;
          idSecuencia++;
          hayRespuesta = false;
          enviarOrdenBrazo(CMD_EJECUTAR, tipoActual);
          cambiarEstado(EST_BRAZO_TRABAJANDO);
        }
      } else if (millis() - tEstado > TIMEOUT_QR_MS) {
        irAError(ERR_QR_TIMEOUT);
      }
      break;

    case EST_BRAZO_TRABAJANDO:
      if (hayRespuesta) {
        hayRespuesta = false;
        if (respId == idSecuencia) {
          if (respEstado == BRAZO_OK) {
            Serial.println("[BRAZO] Secuencia terminada");
            detenerCinta(false);
            cambiarEstado(EST_LIBERAR_CONTENEDOR);
          } else if (respEstado == BRAZO_ERROR) {
            irAError(ERR_BRAZO_FALLA);
          }
        }
      } else if (millis() - tEstado > TIMEOUT_BRAZO_MS) {
        irAError(ERR_BRAZO_TIMEOUT);
      }
      break;

    case EST_LIBERAR_CONTENEDOR:
      if (!sensor.activa) {
        detenerCinta(false);
        cambiarEstado(EST_LIBRE);
      }
      break;

    case EST_ERROR:
      if (botonAck.flanco) {
        Serial.println("[ERROR] Reconocido por el operario");
        causa = ERR_NINGUNO;
        cambiarEstado(EST_LIBERAR_CONTENEDOR);
      }
      break;
  }
}

// SIMULACIÓN (Wokwi)
#if MODO_SIMULACION

void procesarComando(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "FALLA") {
    simFalla = true;
    Serial.println("[SIM] La proxima respuesta del brazo sera ERROR");
  } else if (cmd == "COLGAR") {
    simColgar = true;
    Serial.println("[SIM] El brazo no respondera a la proxima orden");
  } else {
    Serial.printf("[SIM] Texto recibido: %s\n", cmd.c_str());
    if (estado != EST_LEYENDO_QR) {
      Serial.printf("[SIM] Ignorado: el estado actual es %s. Primero apreta SENSOR y espera el LED amarillo.\n", nombreEstado(estado));
    } else {
      qrTipo = tipoDesdeTexto(cmd);
      hayQR  = true;
    }
  }
}

void procesarSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        procesarComando(bufferSerial);
        bufferSerial = "";
      }
    } else {
      bufferSerial += c;
    }
  }
}

void simularBrazo() {
  if (simBrazoActivo && (millis() - simTInicio) > SIM_TIEMPO_BRAZO_MS) {
    simBrazoActivo = false;
    respEstado = simFalla ? BRAZO_ERROR : BRAZO_OK;
    respId = idSecuencia;
    hayRespuesta = true;
    simFalla = false;
    Serial.printf("[SIM] El brazo responde: %s\n", respEstado == BRAZO_OK ? "OK" : "ERROR");
  }
}

#else
// ESP-NOW (hardware real)

bool mismaMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

void procesarMensaje(const uint8_t *mac, const uint8_t *datos, int largo) {
  if (mismaMac(mac, MAC_CAM) && largo == sizeof(MensajeQR)) {
    MensajeQR m;
    memcpy(&m, datos, sizeof(m));
    qrTipo = m.tipo;
    hayQR  = true;
  } else if (mismaMac(mac, MAC_BRAZO) && largo == sizeof(MensajeRespuesta)) {
    MensajeRespuesta m;
    memcpy(&m, datos, sizeof(m));
    respEstado   = m.estado;
    respId       = m.id;
    hayRespuesta = true;
  }
}

// La firma del callback cambió entre versiones del core de ESP32 para Arduino
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void alRecibir(const esp_now_recv_info_t *info, const uint8_t *datos, int largo) {
  procesarMensaje(info->src_addr, datos, largo);
}
#else
void alRecibir(const uint8_t *mac, const uint8_t *datos, int largo) {
  procesarMensaje(mac, datos, largo);
}
#endif

void iniciarEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(alRecibir);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MAC_BRAZO, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Error agregando el brazo como peer");
  }
}
#endif

// SETUP / LOOP

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_LED_VERDE,    OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO,     OUTPUT);
  pinMode(PIN_CINTA_STOP,   OUTPUT);
  pinMode(PIN_BUZZER,       OUTPUT);
  pinMode(PIN_SENSOR,       INPUT_PULLUP);
  pinMode(PIN_BOTON_ACK,    INPUT_PULLUP);

  buzzerApagar();
  detenerCinta(false);

#if MODO_SIMULACION
  Serial.println("ESP32 CENTRAL (SIMULACION)");
  Serial.println("1) Mantene apretado el pulsador SENSOR (contenedor presente)");
  Serial.println("2) Escribi TORNILLO, TUERCA o ARANDELA (simula el QR)");
  Serial.println("Extras: FALLA / COLGAR / cualquier otro texto = QR desconocido");
#else
  iniciarEspNow();
  Serial.println("ESP32 CENTRAL");
#endif

  cambiarEstado(EST_LIBRE);
}

void loop() {
  actualizarEntrada(sensor);
  actualizarEntrada(botonAck);

#if MODO_SIMULACION
  procesarSerial();
  simularBrazo();
#endif

  actualizarEstado();
  actualizarBuzzer();
}
