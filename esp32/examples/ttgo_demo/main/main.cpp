/**
 * This example takes a picture every 5s and print its size on serial monitor.
 */

// =============================== SETUP ======================================

#define BOARD_ESP32CAM_AITHINKER

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

//#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"

// ESP32Cam (AiThinker) PIN Map
#ifdef BOARD_ESP32CAM_AITHINKER

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

//cv::Mat inputImage;

extern "C" {
void app_main(void);
}

static const char *TAG = "example:take_picture";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    //.pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG

    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
};

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

void log_memory() {
    ESP_LOGI("MEMORY", "Free heap size: %d bytes", esp_get_free_heap_size());
    ESP_LOGI("MEMORY", "Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());

    // if (esp_spiram_get_size() > 0) {
    //     ESP_LOGI("MEMORY", "PSRAM total size: %d bytes", esp_spiram_get_size());
    //     ESP_LOGI("MEMORY", "PSRAM free size: %d bytes", esp_spiram_get_free_size());
    // } else {
    //     ESP_LOGI("MEMORY", "PSRAM not enabled");
    // }
}

void app_main()
{
    init_camera();

    while (1) {
        log_memory();
        ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *fb = esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", fb->len);

        cv::Mat inputImage(fb->height, fb->width, CV_8UC2, fb->buf); // rgb565 is 2 channels of 8-bit unsigned

        if (inputImage.empty()) {
            ESP_LOGE(TAG, "Failed to create Mat image");
            esp_camera_fb_return(fb);
            continue;
        }

        ESP_LOGI(TAG, "Image created successfully");

        cvtColor(inputImage, inputImage, cv::COLOR_BGR5652GRAY);
        ESP_LOGI(TAG, "Gray Done");

        ESP_LOGI(TAG, "Before threshold, Free heap size: %d bytes", esp_get_free_heap_size());
        try {
            threshold(inputImage, inputImage, 128, 255, cv::THRESH_BINARY);
        } catch (const cv::Exception& e) {
            ESP_LOGE(TAG, "Exception occurred during threshold: %s", e.what());
            esp_camera_fb_return(fb);
            continue;
        }
        ESP_LOGI(TAG, "Bina done Done");

        esp_camera_fb_return(fb);

        inputImage.release();

        log_memory();

        vTaskDelay(5000 / portTICK_RATE_MS);
    }
}