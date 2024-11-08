// config.h
#ifndef CONFIG_H
#define CONFIG_H

// ***************** Platform configuration *****************
// ******* Define the platform: uncomment either  PLATFORM_REMOTE or PLATFORM_BASE *********
//#define PLATFORM_REMOTE
#define PLATFORM_BASE
#define REMOTE_MAC {0x54, 0x32, 0x04, 0x11, 0xcd, 0x94}         // Replace with actual MAC address of the remote unit
#define BASE_MAC {0x54, 0x32, 0x04, 0x11, 0xca, 0x10}           // Replace with actual MAC address of the base unit
#define DEBUG_TEST  false                                       // Send debug messages for uart and ESP-NOW
#define DEBUG_SEND  false                                       // Send debug messages via ESP-NOW
#define DEBUG_RSSI  false                                       // Print RSSI (signal strength) information
#define RSSI_TASK_DELAY_MS 2000                                 // Delay in milliseconds (e.g., every 2 seconds)


// ***************** WIFI configuration *****************
#define WIFI_CHANNEL                1                   // WiFi Channel (1-13)
#ifdef PLATFORM_REMOTE
    #define RECEIVER_MAC BASE_MAC  
#elif defined(PLATFORM_BASE)
    #define RECEIVER_MAC REMOTE_MAC  
#else
    #error "You must define PLATFORM_REMOTE or PLATFORM_BASE"
#endif    
#define LONG_RANGE                  true                // Enable Long Range Mode
#define EXTERNAL_ANTENNA            true                // Enable external antenna on Seeedstudio Xiao ESP32 C6 on Pin 14
#define EXTERNAL_ANTENNA_PIN        14                  // External antenna pin that gets set to HIGH
#define EXTERNAL_ANTENNA_PIN_ENABLE 3                   // Apparently this pin needs to be set to low in order to enable the external antenna
#define TX_POWER                    84                  // Range is 8-84 which corresponds to 2dBm - 20dBm


// ***************** UART configuration *****************
#ifdef PLATFORM_REMOTE
    #define UART_TX_PIN             1                  // UART TX pin REMOTE 1
    #define UART_RX_PIN             2                  // UART RX pin REMOTE 2
    #define UART_PORT_NUM           UART_NUM_1
#elif defined(PLATFORM_BASE)
    #define UART_TX_PIN             16                  // UART TX pin
    #define UART_RX_PIN             17                  // UART RX pin
    #define UART_PORT_NUM           UART_NUM_0
#endif
#define UART_BAUD_RATE          38400
#define UART_DATA_BITS          UART_DATA_8_BITS
#define UART_PARITY             UART_PARITY_DISABLE
#define UART_STOP_BITS          UART_STOP_BITS_1
#define UART_FLOW_CTRL          UART_HW_FLOWCTRL_DISABLE
#define UART_BUFFER_SIZE        1024
#define UART_QUEUE_SIZE         40                      // Default is 20

// ***************** USB configuration *****************
#define USB_SERIAL_TX_BUFFER_SIZE        1024  // Size of the TX buffer
#define USB_SERIAL_RX_BUFFER_SIZE        1024  // Size of the RX buffer


#endif // CONFIG_H
