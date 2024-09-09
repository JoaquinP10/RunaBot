#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Configura los parámetros de la cámara para el módulo AI Thinker
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char* ssid = "redpucp";
const char* password = "C9AA28BA93";

// Utiliza el puerto 80 para el servidor web
WebServer server(80);

void handle_jpg_stream();
void handleNotFound();
void startCameraServer();

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
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Inicializa la cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error inicializando la cámara: 0x%x", err);
    return;
  }

  // Conéctate a la red WiFi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  // Inicia el servidor de la cámara
  startCameraServer();
}

void loop() {
  server.handleClient();
}

// Inicia el servidor de la cámara
void startCameraServer() {
  server.on("/stream", HTTP_GET, handle_jpg_stream);
  server.onNotFound(handleNotFound);
  server.begin();
}

// Maneja la transmisión de imágenes JPEG
void handle_jpg_stream() {
  WiFiClient client = server.client();
  if (!client) {
    return;
  }

  Serial.println("Iniciando stream");

  // Inicializa el frame buffer
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Error obteniendo el frame buffer");
    return;
  }

  // Envia el encabezado HTTP
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Connection: keep-alive");
  client.println("Cache-Control: no-cache");
  client.println();

  while(true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Error obteniendo el frame buffer");
      break;
    }

    // Envia los datos del frame buffer
    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.println();
    
    esp_camera_fb_return(fb);

    if (!client.connected()) {
      break;
    }
  }
}

// Maneja las solicitudes no encontradas
void handleNotFound(){
  server.send(404, "text/plain", "404: Not Found");
}
