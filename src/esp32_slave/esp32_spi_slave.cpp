/**
 * @file esp32_spi_slave.cpp
 * @brief ESP32-CAM SPI Slave - Arduino/ESP-IDF Framework Pseudo-code
 */

#include "spi_protocol.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_camera.h"

static const char *TAG = "ESP32_SPI_SLAVE";

// --- Camera Configuration ---
// Pin definition for CAMERA_MODEL_AI_THINKER (OV2640)
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

// --- SPI Slave Configuration ---
#define SPI_HOST HSPI_HOST
#define GPIO_CS   15
#define GPIO_SCLK 14
#define GPIO_MOSI 13
#define GPIO_MISO 12

// --- Handshake / Ready Pin ---
// Master should monitor this pin. When HIGH, Slave has data ready in DMA to be clocked out.
#define GPIO_HANDSHAKE    16  // Using IO16 (U2RXD) which is typically free on ESP32-CAM

// Allocate a large DMA buffer to send a whole frame in one shot to avoid Master chunking complexity.
// 60KB should be enough for generic QVGA/VGA JPEG images. 
#define MAX_FRAME_DMA_BUFFER_SIZE 61440 

static uint16_t frame_seq = 0;

// --- Implement CRC exactly matching the Master ---
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

// --- Camera Initialization ---
bool setup_camera() {
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
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG; // OV2640 Hardware JPEG is highly recommended
    config.frame_size = FRAMESIZE_QVGA;   // Default start
    config.jpeg_quality = 12;
    config.fb_count = 2;                  // Use PSRAM for 2 buffers

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return false;
    }
    ESP_LOGI(TAG, "Camera setup successfully");
    return true;
}

// --- SPI Slave Main Task ---
void spi_slave_task(void *pvParameters) {
    // 0. Initialize Handshake Pin
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_HANDSHAKE);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)GPIO_HANDSHAKE, 0); // Default LOW

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = GPIO_MOSI;
    buscfg.miso_io_num = GPIO_MISO;
    buscfg.sclk_io_num = GPIO_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = MAX_FRAME_DMA_BUFFER_SIZE + 100; // Allow huge transfers

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.spics_io_num = GPIO_CS;
    slvcfg.flags = 0;
    slvcfg.queue_size = 3;
    slvcfg.mode = 0; // SPI Mode 0

    // Enable SPI DMA
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI Slave initialized on HSPI.");

    // Buffers in DMA-capable memory (Internal SRAM)
    spi_command_t *cmd_buf = (spi_command_t *)heap_caps_malloc(sizeof(spi_command_t), MALLOC_CAP_DMA);
    spi_ack_t *ack_buf = (spi_ack_t *)heap_caps_malloc(sizeof(spi_ack_t), MALLOC_CAP_DMA);
    spi_frame_header_t *header_buf = (spi_frame_header_t *)heap_caps_malloc(sizeof(spi_frame_header_t), MALLOC_CAP_DMA);
    
    // Allocate large continuous block for the whole image
    uint8_t *frame_dma_buf = (uint8_t *)heap_caps_malloc(MAX_FRAME_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (!frame_dma_buf) {
        ESP_LOGE(TAG, "Failed to allocate 60KB DMA buffer! Try reducing size or checking heap.");
        vTaskDelete(NULL);
    }

    spi_slave_transaction_t trans_cmd, trans_ack, trans_data;

    while (1) {
        // 1. Wait for Master to send a command
        memset(cmd_buf, 0, sizeof(spi_command_t));
        memset(&trans_cmd, 0, sizeof(trans_cmd));
        trans_cmd.length = sizeof(spi_command_t) * 8;
        trans_cmd.rx_buffer = cmd_buf;

        // Block until master transmits
        spi_slave_transmit(SPI_HOST, &trans_cmd, portMAX_DELAY);
        
        ESP_LOGI(TAG, "Command received: %02X", cmd_buf->cmd_id);

        // 2. Process Command & Prepare ACK
        memset(ack_buf, 0, sizeof(spi_ack_t));
        ack_buf->sof = ACK_SOF;
        ack_buf->eof = ACK_EOF;
        ack_buf->cmd_id = cmd_buf->cmd_id;
        ack_buf->seq = cmd_buf->seq;
        ack_buf->status = ACK_OK;

        uint8_t received_crc = cmd_buf->crc8;
        cmd_buf->crc8 = 0; // Zero to verify
        if (received_crc != crc8((uint8_t*)cmd_buf, sizeof(spi_command_t) - 4)) {
            ack_buf->status = ACK_ERR_CRC;
            ESP_LOGW(TAG, "Command CRC Error!");
        }

        bool trigger_capture = false;
        if (ack_buf->status == ACK_OK) {
            switch (cmd_buf->cmd_id) {
                case CMD_ID_SET_FMT:
                    // In a production system, you would call esp_camera_sensor_get()->set_framesize(sensor, framesize) here
                    ESP_LOGI(TAG, "Master requested Format change");
                    break;
                case CMD_ID_CAPTURE_ONCE:
                    ESP_LOGI(TAG, "Master requested CAPTURE_ONCE");
                    trigger_capture = true;
                    break;
                default:
                    ESP_LOGI(TAG, "Unhandled Command ID");
                    ack_buf->status = ACK_ERR_CMD;
                    break;
            }
        }

        ack_buf->crc8 = crc8((uint8_t*)ack_buf, sizeof(spi_ack_t) - 2);

        // 3. Send ACK to Master
        memset(&trans_ack, 0, sizeof(trans_ack));
        trans_ack.length = sizeof(spi_ack_t) * 8;
        trans_ack.tx_buffer = ack_buf;
        spi_slave_transmit(SPI_HOST, &trans_ack, portMAX_DELAY);

        // 4. Handle requested data streams
        if (trigger_capture) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGE(TAG, "Camera capture failed");
            } else {
                ESP_LOGI(TAG, "Captured! Size: %zu bytes", fb->len);

                // Prepare Frame Header
                memset(header_buf, 0, sizeof(spi_frame_header_t));
                header_buf->sof[0] = FRAME_SOF1; header_buf->sof[1] = FRAME_SOF2;
                header_buf->eof[0] = FRAME_EOF1; header_buf->eof[1] = FRAME_EOF2;
                header_buf->ver = PROTOCOL_VERSION;
                header_buf->flags = (fb->format == PIXFORMAT_JPEG) ? FRAME_FLAG_JPEG : 0;
                header_buf->seq = frame_seq++;
                header_buf->width = fb->width;
                header_buf->height = fb->height;
                header_buf->pixfmt = PIXFMT_JPEG; // Or mapping from fb->format
                header_buf->frame_size = fb->len;
                header_buf->timestamp = esp_timer_get_time() / 1000;
                header_buf->header_crc32 = crc32((uint8_t*)header_buf, 20);

                // Transmit Header
                memset(&trans_data, 0, sizeof(trans_data));
                trans_data.length = sizeof(spi_frame_header_t) * 8;
                trans_data.tx_buffer = header_buf;
                
                // Set READY pin HIGH to tell master "Header is ready to be clocked out"
                gpio_set_level((gpio_num_t)GPIO_HANDSHAKE, 1);
                spi_slave_transmit(SPI_HOST, &trans_data, portMAX_DELAY);
                gpio_set_level((gpio_num_t)GPIO_HANDSHAKE, 0); // Pull LOW after master reads

                // Transmit Image payload in one single DMA shot (if it fits)
                if (fb->len <= MAX_FRAME_DMA_BUFFER_SIZE) {
                    // Copy all PSRAM data to the DMA-safe contiguous buffer
                    memcpy(frame_dma_buf, fb->buf, fb->len);
                    
                    // Critical: Length must be padded to a 4-byte boundary to avoid ESP32 SPI Slave hardware crashing!
                    size_t padded_len = (fb->len + 3) & ~3;

                    memset(&trans_data, 0, sizeof(trans_data));
                    trans_data.length = padded_len * 8; // length is in bits
                    trans_data.tx_buffer = frame_dma_buf;
                    
                    // Set READY pin HIGH to tell master "Data payload is ready!"
                    gpio_set_level((gpio_num_t)GPIO_HANDSHAKE, 1);
                    spi_slave_transmit(SPI_HOST, &trans_data, portMAX_DELAY);
                    gpio_set_level((gpio_num_t)GPIO_HANDSHAKE, 0); // Pull LOW after transmission is done
                    
                    ESP_LOGI(TAG, "Frame sent successfully in one block.");
                } else {
                    ESP_LOGE(TAG, "Frame size (%zu) exceeds DMA buffer size (%d)! Dropping frame.", fb->len, MAX_FRAME_DMA_BUFFER_SIZE);
                    // In a production system, you might want to either increase buffer size, 
                    // compress the jpeg more, or implement a multi-chunk protocol.
                }

                esp_camera_fb_return(fb);
            }
        }
    }
}

// --- App Main (FreeRTOS Entry Point) ---
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "System starting");

    if (!setup_camera()) {
        ESP_LOGE(TAG, "Camera init failed. Halting.");
        while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }

    // Pin task to core 1 yielding highest SPI reliability
    xTaskCreatePinnedToCore(spi_slave_task, "spi_slave_task", 4096 * 2, NULL, 5, NULL, 1);
}

