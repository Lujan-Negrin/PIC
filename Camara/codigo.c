
// ESP32-CAM: mide el color del centro de la imagen.
// Conexiones:
//   Camara OV3660 integrada en la placa AI Thinker ESP32-CAM
//   Alimentacion y programacion por USB-serie (115200)
//==========================================================

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

// Exposicion y ganancia fijas: tienen que ser las mismas que en el sketch de calibracion
#define EXPOSICION_FIJA     1
#define EXPOSICION_VALOR  600
#define GANANCIA_VALOR      5

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

void setup() {
  Serial.begin(115200);
  Serial.println();
  if (!iniciarCamara()) {
    Serial.println("[CAM] Sin camara, revisar la conexion y reiniciar");
    while (true) delay(1000);
  }
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
  esp_camera_fb_return(fb);
  Serial.printf("R=%.0f G=%.0f B=%.0f\n", r, g, b);
  delay(150);
}
