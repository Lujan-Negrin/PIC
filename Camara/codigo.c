
// ESP32-CAM: clasifica el color del contenedor (gris, naranja, beige) y detecta si hay un contenedor.
// Conexiones:
//   Camara OV3660 integrada en la placa AI Thinker ESP32-CAM
//   Alimentacion y programacion por USB-serie (115200)
//==========================================================

#include <math.h>
#include "esp_camera.h"
#include <string.h>
#include <stdint.h>

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

// Iguales a los valores PIEZA_* de la central y del brazo
#define PIEZA_DESCONOCIDA 0
#define PIEZA_TORNILLO    1
#define PIEZA_TUERCA      2
#define PIEZA_ARANDELA    3

// Exposicion y ganancia fijas: tienen que ser las mismas que en el sketch de calibracion
#define EXPOSICION_FIJA     1
#define EXPOSICION_VALOR  600
#define GANANCIA_VALOR      5

// Fondo vacio medido en la calibracion (1 = usarlo, 0 = medirlo al prender la camara)
#define USAR_FONDO_FIJO            1
#define FONDO_R                   36
#define FONDO_G                   67
#define FONDO_B                   39

// Diferencia minima respecto del fondo vacio para decir que hay un contenedor
#define UMBRAL_PRESENCIA          60

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

static const char *nombrePieza(uint8_t tipo) {
  switch (tipo) {
    case PIEZA_TORNILLO: return "TORNILLO (gris)";
    case PIEZA_TUERCA:   return "TUERCA (naranja)";
    case PIEZA_ARANDELA: return "ARANDELA (beige)";
    default:             return "DESCONOCIDO (mezcla / en transito, se ignora)";
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  if (!iniciarCamara()) {
    Serial.println("[CAM] Sin camara, revisar la conexion y reiniciar");
    while (true) delay(1000);
  }
  delay(2000);   // calentamiento: los primeros cuadros salen inestables
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == NULL) {
    Serial.println("[CAM] Error capturando frame");
    delay(200);
    return;
  }
  float r, g, b;
  colorPromedioCentral(fb, &r, &g, &b);
  uint8_t tipo = clasificarColor(r, g, b);
  esp_camera_fb_return(fb);

  float dif = fabsf(r - FONDO_R) + fabsf(g - FONDO_G) + fabsf(b - FONDO_B);
  bool hayContenedor = (dif > UMBRAL_PRESENCIA);
  Serial.printf("R=%.0f G=%.0f B=%.0f  dif=%.0f  -> %s\n", r, g, b, dif,
                hayContenedor ? nombrePieza(tipo) : "(vacio, no hay contenedor)");
  delay(150);
}
