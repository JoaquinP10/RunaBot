#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Configura los parámetros de la cámara para el módulo AI Thinker
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Define el SSID y la contraseña del AP
const char *ssid = "ESP32-CAM-AP";
const char *password = "12345678";

// Página web HTML con JavaScript para actualizar la imagen
const char* html = R"rawliteral(
<html>
  <head>
    <title>ESP32-CAM Stream</title>
  </head>
  <body>
    <h1>ESP32-CAM Stream</h1>
    <img src="capture" id="stream" />
    <script>
      setInterval(function() {
        document.getElementById('stream').src = 'capture?rand=' + Math.random();
      }, 1000);
    </script>
  </body>
</html>
)rawliteral";

// Inicializa el servidor web en el puerto 80
WebServer server(80);

void startCameraServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", html);
  });

  server.on("/capture", HTTP_GET, []() {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      server.send(500, "text/plain", "Error capturando la imagen");
      return;
    }

    server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  });

  server.begin();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // Cambiar a PIXFORMAT_GRAYSCALE si es necesario

  // Reducir el tamaño de la imagen para mejorar los FPS
  config.frame_size = FRAMESIZE_QVGA; // Cambiar a FRAMESIZE_QQVGA para aún más FPS
  config.jpeg_quality = 12; // Reducir la calidad JPEG para aumentar los FPS
  config.fb_count = 2;

  // Inicializa la cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error inicializando la cámara: 0x%x", err);
    return;
  }

  Serial.println("Cámara inicializada correctamente");

  // Configura el AP
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Inicia el servidor web
  startCameraServer();
}

void loop() {
  server.handleClient();
}
