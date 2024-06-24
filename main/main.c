#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "nvs_flash.h"
#include "config.h"

#ifdef PLATFORM_REMOTE
    #define TAG "UART_BRIDGE_REMOTE"  
#elif defined(PLATFORM_BASE)
    #define TAG "UART_BRIDGE_BASE"  
#endif


// Define the receiver MAC address as a global variable
static uint8_t receiver_mac[] = RECEIVER_MAC;

// Queue to handle uart
static QueueHandle_t uart_queue;
// Queue to hold received ESP-NOW data
static QueueHandle_t esp_now_queue;

// Structure to hold received ESP-NOW data
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t data[UART_BUFFER_SIZE];
    int len;
} esp_now_packet_t;

// Corrected callback when data is received via ESP-NOW
static void esp_now_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len) {
    esp_now_packet_t packet;
    if (len > UART_BUFFER_SIZE) {
        len = UART_BUFFER_SIZE; // truncate if data exceeds buffer
    }
    memcpy(packet.mac_addr, esp_now_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(packet.data, data, len);
    packet.len = len;
    if (xQueueSend(esp_now_queue, &packet, portMAX_DELAY) != pdPASS) {
        ESP_LOGW(TAG, "Failed to enqueue received packet");
    }
}

// Callback when data is sent via ESP-NOW
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}

// Initialize UART
void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_BITS,
        .parity    = UART_PARITY,
        .stop_bits = UART_STOP_BITS,
        .flow_ctrl = UART_FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 20, &uart_queue, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Initialize Wifi
void init_wifi() {
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
        
    // Enable long-range mode if configured
    if (LONG_RANGE) {
        ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR));
    }
}

// Initialize ESP-NOW
void init_esp_now() {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));

    // Add peer information
    esp_now_peer_info_t peer_info = {
        .peer_addr = RECEIVER_MAC,
        .channel = WIFI_CHANNEL,
        .encrypt = false,
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}

// Set predefined GPIO to high for the use of an external antenna, i.e. GPIO14 Xiao C6
void configure_external_antenna(void) {                         
    esp_rom_gpio_pad_select_gpio(EXTERNAL_ANTENNA_PIN);
    gpio_set_direction(EXTERNAL_ANTENNA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EXTERNAL_ANTENNA_PIN, 1);  
}

// Task to read from UART and send via ESP-NOW
void uart_to_esp_now_task(void *pvParameter) {
    uint8_t data[UART_BUFFER_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUFFER_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (DEBUG_TEST) {
                ESP_LOGI(TAG, "uart received, sending");
                ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);                    // Print data as a hex dump into serial
            }
            ESP_ERROR_CHECK(esp_now_send(receiver_mac, data, len));                     // Send data via ESP-NOW
        }
    }
}

// Task to read from the ESP-NOW queue and write to UART
void esp_now_to_uart_task(void *pvParameter) {
    esp_now_packet_t packet;
    while (1) {
        if (xQueueReceive(esp_now_queue, &packet, portMAX_DELAY) == pdPASS) {
            uart_write_bytes(UART_PORT_NUM, (const char *)packet.data, packet.len);
            if (DEBUG_TEST) {
                ESP_LOGI(TAG, "ESPNOW packet received, hex dump:");
                ESP_LOG_BUFFER_HEXDUMP(TAG, packet.data, packet.len, ESP_LOG_INFO);                    // Print data as a hex dump into serial
                ESP_LOGI(TAG, "Data received: %.*s", packet.len, packet.data);                         // Print data as a string
            }
        }
    }
}

// Debug task to send 
void send_debug_task(void *pvParameter) {
    const char *message = "Hello from ESP32-C6!";
    while (1) {
        ESP_LOGI(TAG, "Sending message: %s", message);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Send every 1 second
        uart_write_bytes(UART_PORT_NUM, (const char *)message, strlen(message));
    }
}


void app_main() {
    ESP_LOGI(TAG, "Starting ESPNOW UART BRIDGE in 2 seconds");
    vTaskDelay(pdMS_TO_TICKS(2000));
    //esp_err_t ret = nvs_flash_init();
    nvs_flash_init();
    esp_now_queue = xQueueCreate(10, sizeof(esp_now_packet_t));  // Create queue to hold received ESP-NOW packets
    if (esp_now_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create xQueue");
        return;
    }
    else {
        ESP_LOGI(TAG, "xQueue created");
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    init_uart();
    ESP_LOGI(TAG, "Uart initialized");
        if (EXTERNAL_ANTENNA) {             //configured in config.h
        configure_external_antenna();
        ESP_LOGI(TAG, "External antenna configured");
    }
    init_wifi();
    ESP_LOGI(TAG, "Wifi initialized");
    init_esp_now();
    ESP_LOGI(TAG, "ESPNOW initialized");
    ESP_LOGI(TAG, "Starting xTasks in 2 seconds");
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreate(uart_to_esp_now_task, "uart_to_esp_now_task", 4096, NULL, 5, NULL);
    xTaskCreate(esp_now_to_uart_task, "esp_now_to_uart_task", 4096, NULL, 5, NULL);
    //xTaskCreate(send_debug_task, "send_debug_task", 4096, NULL, 5, NULL);
}
