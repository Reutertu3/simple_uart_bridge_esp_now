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
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"
#include <fcntl.h>

#ifdef PLATFORM_REMOTE
    #define TAG "UART_BRIDGE_REMOTE"  
    static const uint8_t own_mac_address[6] = REMOTE_MAC; // Initialize with the #define value1
#elif defined(PLATFORM_BASE)
    #define TAG "UART_BRIDGE_BASE"  
    static const uint8_t own_mac_address[6] = BASE_MAC; // Initialize with the #define value1
#endif



// Define the receiver MAC address as a global variable
static uint8_t receiver_mac[6] = RECEIVER_MAC;
static const uint8_t peer_mac_address[6] = RECEIVER_MAC; // Initialize with the #define value
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



// Callback function for when data is received via ESP-NOW
static void esp_now_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len) {
    esp_now_packet_t packet;
    if (len > UART_BUFFER_SIZE) {
        len = UART_BUFFER_SIZE; // truncate if data exceeds buffer
    }
    memcpy(packet.mac_addr, esp_now_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(packet.data, data, len);
    packet.len = len;

    #if DEBUG_RSSI
    // Log the RSSI value
    int8_t rssi = esp_now_info->rx_ctrl->rssi;  // Retrieves the Received Signal Strength Indicator (RSSI) from the esp_now_info struct
    ESP_LOGI(TAG, "Received data from MAC: " MACSTR ", RSSI: %d dBm, Data length: %d",
             MAC2STR(esp_now_info->src_addr), rssi, len);
    #endif
    if (xQueueSend(esp_now_queue, &packet, portMAX_DELAY) != pdPASS) {
        ESP_LOGW(TAG, "Failed to enqueue received packet");
    }
}



// Callback when data is sent via ESP-NOW
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    #if DEBUG_TEST
        ESP_LOGI(TAG, "Send status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
    #endif
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

    uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE * 4, UART_BUFFER_SIZE * 4, UART_QUEUE_SIZE, &uart_queue, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    #ifdef PLATFORM_REMOTE
        uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    #endif

    
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
    esp_wifi_set_max_tx_power(TX_POWER);
}



// Initialize and install the USB Serial/JTAG driver
void init_usb() {
    // Create and configure the USB Serial/JTAG driver configuration
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = USB_SERIAL_TX_BUFFER_SIZE, // Set TX buffer size
        .rx_buffer_size = USB_SERIAL_RX_BUFFER_SIZE, // Set RX buffer size
    };

    // Install the USB Serial/JTAG driver
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        ESP_LOGE("USB_SERIAL_JTAG", "Failed to install USB Serial/JTAG driver: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI("USB_SERIAL_JTAG", "USB Serial/JTAG driver installed successfully");
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
    esp_rom_gpio_pad_select_gpio(EXTERNAL_ANTENNA_PIN_ENABLE);
    gpio_set_direction(EXTERNAL_ANTENNA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(EXTERNAL_ANTENNA_PIN_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(EXTERNAL_ANTENNA_PIN_ENABLE, 0);
    gpio_set_level(EXTERNAL_ANTENNA_PIN, 1);   
}



// Task to read from UART or jtag and send via ESP-NOW
void uart_to_esp_now_task(void *pvParameter) {
    uint8_t data[UART_BUFFER_SIZE];
    while (1) {
        #ifdef PLATFORM_REMOTE
            int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUFFER_SIZE, 20 / portTICK_PERIOD_MS);
        #elif defined(PLATFORM_BASE)
            int len = usb_serial_jtag_read_bytes(data, sizeof(data), portTICK_PERIOD_MS);
        #endif
        
        if (len > 0) {
            ESP_ERROR_CHECK(esp_now_send(receiver_mac, data, len));                     // Send data via ESP-NOW
            #if DEBUG_TEST
                ESP_LOGI(TAG, "uart received, sending");
                ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);                    // Print data as a hex dump into serial
            #endif

        }
    }
}



// Task to read from the ESP-NOW queue and write to either UART or jtag, depending on the platform
void esp_now_to_uart_task(void *pvParameter) {
    esp_now_packet_t packet;
    while (1) {
        if (xQueueReceive(esp_now_queue, &packet, portMAX_DELAY) == pdPASS) {           
            #ifdef PLATFORM_REMOTE
                uart_write_bytes(UART_PORT_NUM, (const char *)packet.data, packet.len);
            #elif defined(PLATFORM_BASE)
                usb_serial_jtag_write_bytes(packet.data, packet.len, portMAX_DELAY);
            #endif
            #if DEBUG_TEST
                ESP_LOGI(TAG, "ESPNOW packet received, hex dump:");
                ESP_LOG_BUFFER_HEXDUMP(TAG, packet.data, packet.len, ESP_LOG_INFO);                    // Print data as a hex dump into serial
                ESP_LOGI(TAG, "Data received: %.*s", packet.len, packet.data);                         // Print data as a string
            #endif
        }
    }
}



// Debug task to send periodic messages to UART
void send_debug_task(void *pvParameter) {
    const char *message = "Hello from ESP32-C6!";
    while (1) {
        ESP_LOGI(TAG, "Sending message: %s", message);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Send every 2 seconds
        uart_write_bytes(UART_PORT_NUM, (const char *)message, strlen(message));
    }
}



// Print out peer mac address upon app start
void log_mac_addresses() {
    char peer_mac_str[18]; // Buffer to hold the MAC address string
    char own_mac_str[18]; // Hold own mac in String

    // Convert the MAC address to a string
    snprintf(peer_mac_str, sizeof(peer_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             peer_mac_address[0], peer_mac_address[1], peer_mac_address[2],
             peer_mac_address[3], peer_mac_address[4], peer_mac_address[5]);
    snprintf(own_mac_str, sizeof(own_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             own_mac_address[0], own_mac_address[1], own_mac_address[2],
             own_mac_address[3], own_mac_address[4], own_mac_address[5]);

    // Log the MAC address
    ESP_LOGI("TAG", "Own MAC: %s", own_mac_str);
    ESP_LOGI("TAG", "Configured peer MAC: %s", peer_mac_str);
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
    init_uart();                        //Initialize UART with params from config.h
    ESP_LOGI(TAG, "Uart initialized");
    if (EXTERNAL_ANTENNA) {             //configured in config.h
        configure_external_antenna();
        ESP_LOGI(TAG, "External antenna configured");
    }
    init_wifi();                        //Initialize WIFI with params from config.h 
    init_usb();                         //Initialize USB with params from config.h
    ESP_LOGI(TAG, "Wifi initialized");
    init_esp_now();                     //Initialize ESP-NOW with params from config.h
    ESP_LOGI(TAG, "ESPNOW initialized");
    log_mac_addresses();                //Print out condfigured MAC adresses 
    ESP_LOGI(TAG, "Starting xTasks in 2 seconds");
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreate(uart_to_esp_now_task, "uart_to_esp_now_task", 4096, NULL, 5, NULL);
    xTaskCreate(esp_now_to_uart_task, "esp_now_to_uart_task", 4096, NULL, 5, NULL);
        
    #if DEBUG_SEND
    xTaskCreate(send_debug_task, "send_debug_task", 4096, NULL, 5, NULL);
    #endif

}
