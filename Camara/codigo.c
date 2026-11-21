// Conexiones:
//   Camara OV3660 integrada en la placa AI Thinker ESP32-CAM
//   GPIO13 sensor de entrada (opcional, sin uso en la version final)
//   La luz blanca esta conectada a la central, no a esta placa
//==========================================================

// ESP32-CAM: detecta el color del contenedor (gris = tornillo, naranja = tuerca,
// beige = arandela) y se lo manda a la central por ESP-NOW.
// Placa: AI Thinker ESP32-CAM. Para cargar el codigo: GPIO0 a GND (o boton IO0) mientras dice "Connecting...".
// La camara queda siempre prendida (TIEMPO_CAMARA_ENCENDIDA_MS = 0). Con un sensor de entrada
// en GPIO13 podria apagarse sola despues de un tiempo sin actividad.
#include <math.h>
#include "esp_camera.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Pines de la camara (AI Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Sensor de entrada (opcional, solo se usa si TIEMPO_CAMARA_ENCENDIDA_MS > 0)
#define PIN_SENSOR_ENTRADA 13

// MAC de la central
static uint8_t macCentral[6] = {0x7C, 0xD5, 0x44, 0x24, 0x55, 0x08};

// Iguales a los valores PIEZA_* de la central y del brazo
#define PIEZA_DESCONOCIDA 0
#define PIEZA_TORNILLO    1
#define PIEZA_TUERCA      2
#define PIEZA_ARANDELA    3

// Umbrales de color
#define UMBRAL_G_OSCURO            35
#define UMBRAL_B_CONTENEDOR_NUEVO  15
#define UMBRAL_AMARILLEZ_NARANJA   50
#define LECTURAS_PARA_CONFIRMAR     3
#define TIEMPO_ENTRE_ENVIOS      1500

// Tiempo que queda prendida la camara sin actividad (0 = nunca se apaga)
#define TIEMPO_CAMARA_ENCENDIDA_MS  0

// Deteccion de contenedor: calentamiento de la camara y fondo vacio de referencia
#define TIEMPO_CALENTAMIENTO_MS   2000

#define FRAMES_BASELINE            4

// Fondo vacio medido en la calibracion (1 = usarlo, 0 = medirlo al prender la camara)
#define USAR_FONDO_FIJO            1
#define FONDO_R                   36
#define FONDO_G                   67
#define FONDO_B                   39

// 1 = la camara arranca prendida
#define ARRANCA_PRENDIDA           1

// Diferencia minima respecto del fondo vacio para decir que hay un contenedor
#define UMBRAL_PRESENCIA          60

#define TIEMPO_DEBOUNCE_SENSOR_MS 30

// Exposicion y ganancia fijas: tienen que ser las mismas que en el sketch de calibracion
#define EXPOSICION_FIJA     1
#define EXPOSICION_VALOR  600
#define GANANCIA_VALOR      5

// Estado global
static uint8_t ultimaDeteccion = PIEZA_DESCONOCIDA;
static uint8_t repeticiones = 0;
static unsigned long ultimoEnvio = 0;

static bool camaraEncendida = false;
static unsigned long ultimaActividad = 0;

static unsigned long tArranque = 0;
static uint8_t framesBaseline = 0;
static float baseR = 0, baseG = 0, baseB = 0;

static bool sensorActivo = false;
static bool sensorLecturaPrev = false;
static unsigned long sensorTCambio = 0;

static const char *nombrePieza(uint8_t tipo) {
  switch (tipo) {
    case PIEZA_TORNILLO: return "TORNILLO (gris)";
    case PIEZA_TUERCA:   return "TUERCA (naranja)";
    case PIEZA_ARANDELA: return "ARANDELA (beige)";
    default:             return "DESCONOCIDO (mezcla / en transito, se ignora)";
  }
}

static bool sensorEntradaFlancoActivacion(void) {
  bool lectura = (digitalRead(PIN_SENSOR_ENTRADA) == LOW);
  if (lectura != sensorLecturaPrev) {
    sensorTCambio = millis();
    sensorLecturaPrev = lectura;
  }
  if ((millis() - sensorTCambio) > TIEMPO_DEBOUNCE_SENSOR_MS && lectura != sensorActivo) {
    sensorActivo = lectura;
    if (sensorActivo) {
      return true;
    }
  }
  return false;
}

static bool iniciarCamara(void) {
  camera_config_t config;
  memset(&config, 0, sizeof(config));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QQVGA;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Error iniciando la camara: 0x%x\n", err);
    return false;
  }

#if EXPOSICION_FIJA
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_exposure_ctrl(s, 0);
    s->set_aec2(s, 0);
    s->set_aec_value(s, EXPOSICION_VALOR);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, GANANCIA_VALOR);
  }
#endif
  return true;
}

static void prenderCamara(void) {
  if (camaraEncendida) {
    return;
  }
  if (iniciarCamara()) {
    camaraEncendida = true;
    ultimaDeteccion = PIEZA_DESCONOCIDA;
    repeticiones = 0;
    tArranque = millis();
#if USAR_FONDO_FIJO
    framesBaseline = FRAMES_BASELINE;
    baseR = FONDO_R; baseG = FONDO_G; baseB = FONDO_B;
#else
    framesBaseline = 0;
    baseR = 0; baseG = 0; baseB = 0;
#endif
    Serial.println("[CAM] Camara ENCENDIDA (detecto el sensor de entrada)");
  } else {
    Serial.println("[CAM] No se pudo encender la camara");
  }
}

static void apagarCamara(void) {
  if (!camaraEncendida) {
    return;
  }
  esp_camera_deinit();

  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH);
  camaraEncendida = false;
  Serial.println("[CAM] Camara APAGADA (sin actividad, ahorrando y enfriando)");
}

// Promedio del color de un cuadrado de 40x40 en el centro de la imagen
static void colorPromedioCentral(camera_fb_t *fb, float *rProm, float *gProm, float *bProm) {
  int w = fb->width;
  int h = fb->height;
  int tam = 40;
  int x0 = (w - tam) / 2;
  int y0 = (h - tam) / 2;

  uint16_t *pixeles = (uint16_t *)fb->buf;
  long sumaR = 0, sumaG = 0, sumaB = 0;
  long cuenta = 0;

  for (int y = y0; y < y0 + tam; y++) {
    for (int x = x0; x < x0 + tam; x++) {
      uint16_t p = pixeles[y * w + x];
      // orden de bytes invertido: es el que da colores correctos en la calibracion
      p = (p >> 8) | (p << 8);

      uint8_t r5 = (p >> 11) & 0x1F;
      uint8_t g6 = (p >> 5) & 0x3F;
      uint8_t b5 = p & 0x1F;
      sumaR += (r5 * 255) / 31;
      sumaG += (g6 * 255) / 63;
      sumaB += (b5 * 255) / 31;
      cuenta++;
    }
  }
  *rProm = (float)sumaR / cuenta;
  *gProm = (float)sumaG / cuenta;
  *bProm = (float)sumaB / cuenta;
}

// Colores de referencia medidos. Un color solo se acepta si esta cerca de una
// referencia: las mezclas de cuando el contenedor entra o sale se ignoran (DESCONOCIDO).
#define REF_GRIS_R   119
#define REF_GRIS_G   202
#define REF_GRIS_B   151
#define REF_NARANJA_R 154
#define REF_NARANJA_G 127
#define REF_NARANJA_B  57
#define REF_BEIGE_R   20
#define REF_BEIGE_G   32
#define REF_BEIGE_B    0
#define DIST_MAX_GRIS     50
#define DIST_MAX_NARANJA  60
#define DIST_MAX_BEIGE    40

static float distanciaColor(float r, float g, float b, float rr, float gg, float bb) {
  float dr = r - rr, dg = g - gg, db = b - bb;
  return sqrtf(dr * dr + dg * dg + db * db);
}

static uint8_t clasificarColor(float r, float g, float b) {

  // el gris refleja la luz y satura la imagen: ningun otro contenedor llega a eso
  if (r > 150 && g > 235 && b > 200) return PIEZA_TORNILLO;
  float dGris    = distanciaColor(r, g, b, REF_GRIS_R, REF_GRIS_G, REF_GRIS_B);
  float dNaranja = distanciaColor(r, g, b, REF_NARANJA_R, REF_NARANJA_G, REF_NARANJA_B);
  float dBeige   = distanciaColor(r, g, b, REF_BEIGE_R, REF_BEIGE_G, REF_BEIGE_B);

  uint8_t mejor = PIEZA_DESCONOCIDA;
  float dMejor = 1e9f;
  if (dGris < DIST_MAX_GRIS && dGris < dMejor)          { mejor = PIEZA_TORNILLO; dMejor = dGris; }
  if (dNaranja < DIST_MAX_NARANJA && dNaranja < dMejor) { mejor = PIEZA_TUERCA;   dMejor = dNaranja; }
  if (dBeige < DIST_MAX_BEIGE && dBeige < dMejor)       { mejor = PIEZA_ARANDELA; dMejor = dBeige; }
  return mejor;
}

// Votacion por cada paso de contenedor.
#define VOTOS_PARA_CONFIRMAR  3
#define VOTOS_AL_SALIR        2
#define VOTOS_BEIGE_MIN       3
#define VOTOS_BEIGE_SI_HAY_OTRO 6
#define MUESTREO_MS          30

static uint8_t votos[4] = {0, 0, 0, 0};
static bool enviadoEnEstePaso = false;
static uint8_t vaciosSeguidos = 0;
static uint8_t desconocidos = 0;
#define DESCONOCIDOS_MIN      4
#define DESCONOCIDOS_VS_VOTOS 3
#define VACIOS_PARA_CERRAR    4
static uint8_t ultimoTipoEnviado = PIEZA_DESCONOCIDA;

static void enviarColor(uint8_t tipo) {

  if (tipo == ultimoTipoEnviado && (millis() - ultimoEnvio) <= TIEMPO_ENTRE_ENVIOS) return;
  uint8_t msg[1];
  msg[0] = tipo;
  esp_now_send(macCentral, msg, 1);
  Serial.print("[CAM] Color confirmado y enviado: ");
  Serial.println(nombrePieza(tipo));
  ultimoEnvio = millis();
  ultimoTipoEnviado = tipo;
  ultimaActividad = millis();
  enviadoEnEstePaso = true;
}

static void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("[CAM] Enviado OK");
  } else {
    Serial.println("[CAM] Error al enviar");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(PIN_SENSOR_ENTRADA, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("Mi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println("(copiar esta MAC en MAC_CAM del codigo de la central)");

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, macCentral, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error agregando a la central como peer");
  }

  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH);
  camaraEncendida = false;
  ultimaActividad = millis();
#if ARRANCA_PRENDIDA
  prenderCamara();
  Serial.println("Camara prendida al arrancar; el sensor de entrada la vuelve a prender si se apaga por inactividad.");
#else
  Serial.println("Camara apagada; el sensor de entrada la prende cuando pasa un contenedor.");
#endif
}

void loop() {

  if (sensorEntradaFlancoActivacion()) {
    Serial.println("[SENSOR ENTRADA] activado");
    ultimaActividad = millis();
    prenderCamara();
  }

  if (!camaraEncendida) {
    delay(20);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == NULL) {
    Serial.println("[CAM] Error capturando frame");
    delay(200);
  } else {
    float r, g, b;
    colorPromedioCentral(fb, &r, &g, &b);
    uint8_t tipo = clasificarColor(r, g, b);

    esp_camera_fb_return(fb);

    // Hay contenedor adelante o solo fondo vacio
    bool hayContenedor = false;
    bool calentando = ((millis() - tArranque) < TIEMPO_CALENTAMIENTO_MS);
    if (calentando) {
      Serial.printf("[CAM] calentando t=%lu ms  R=%.0f G=%.0f B=%.0f\n", millis() - tArranque, r, g, b);
    } else if (framesBaseline < FRAMES_BASELINE) {
      baseR += r; baseG += g; baseB += b;
      framesBaseline++;
      if (framesBaseline == FRAMES_BASELINE) {
        baseR /= FRAMES_BASELINE; baseG /= FRAMES_BASELINE; baseB /= FRAMES_BASELINE;
        Serial.printf("[CAM] fondo vacio guardado: R=%.0f G=%.0f B=%.0f. Listo para detectar.\n", baseR, baseG, baseB);
      }
    } else {
      float dif = fabsf(r - baseR) + fabsf(g - baseG) + fabsf(b - baseB);
      hayContenedor = (dif > UMBRAL_PRESENCIA);
      Serial.printf("R=%.0f G=%.0f B=%.0f  dif=%.0f  -> %s\n", r, g, b, dif,
                    hayContenedor ? nombrePieza(tipo) : "(vacio, no hay contenedor)");
    }

    // Votacion: se cuentan los votos de cada color mientras el contenedor esta adelante;
    // se manda el color al juntar suficientes votos o cuando el contenedor se va
    if (!hayContenedor) {

      if (vaciosSeguidos < 255) vaciosSeguidos++;
      if (vaciosSeguidos < VACIOS_PARA_CERRAR) {

      } else {
      if (!enviadoEnEstePaso && (votos[1] + votos[2] + votos[3]) > 0) {
        Serial.printf("[CAM] votos: gris=%d naranja=%d beige=%d\n", votos[1], votos[2], votos[3]);
        uint8_t claro = (votos[1] >= votos[2]) ? PIEZA_TORNILLO : PIEZA_TUERCA;
        uint8_t vClaro = votos[claro];
        if (votos[3] >= VOTOS_BEIGE_MIN && votos[3] >= 2 * vClaro) {
          enviarColor(PIEZA_ARANDELA);
        } else if (vClaro >= VOTOS_AL_SALIR) {
          enviarColor(claro);
        } else if (votos[3] >= (vClaro > 0 ? VOTOS_BEIGE_SI_HAY_OTRO : VOTOS_BEIGE_MIN)) {
          enviarColor(PIEZA_ARANDELA);
        }
      }

      // Contenedor de otro color: se avisa a la central como DESCONOCIDO
      if (!enviadoEnEstePaso && desconocidos >= DESCONOCIDOS_MIN &&
          desconocidos >= DESCONOCIDOS_VS_VOTOS * (votos[1] + votos[2] + votos[3])) {
        Serial.printf("[CAM] color desconocido: %d lecturas raras, %d votos validos\n", desconocidos, votos[1] + votos[2] + votos[3]);
        enviarColor(PIEZA_DESCONOCIDA);
      }
      votos[1] = votos[2] = votos[3] = 0;
      desconocidos = 0;
      enviadoEnEstePaso = false;
      }
    } else {
      vaciosSeguidos = 0;
    }
    if (hayContenedor && tipo == PIEZA_DESCONOCIDA && desconocidos < 255) desconocidos++;
    if (hayContenedor && tipo != PIEZA_DESCONOCIDA) {
      votos[tipo]++;

      if (tipo != PIEZA_ARANDELA && !enviadoEnEstePaso && votos[tipo] >= VOTOS_PARA_CONFIRMAR) enviarColor(tipo);
    }

    delay(calentando ? 20 : MUESTREO_MS);
  }

  if (TIEMPO_CAMARA_ENCENDIDA_MS > 0 && (millis() - ultimaActividad) > TIEMPO_CAMARA_ENCENDIDA_MS) {
    apagarCamara();
  }
}