// config.h
#ifndef CONFIG_H
#define CONFIG_H

// ***************** Platform configuration *****************
// Define the platform: uncomment one of these lines
#define PLATFORM_REMOTE
//#define PLATFORM_BASE
#define REMOTE_MAC {0x54, 0x32, 0x04, 0x11, 0xcd, 0x94}     // Replace with actual MAC address of the remote unit
#define BASE_MAC {0x54, 0x32, 0x04, 0x11, 0xca, 0x10}       // Replace with actual MAC address of the base unit
#define DEBUG_TEST true


// ***************** WIFI configuration *****************
#define WIFI_CHANNEL            1                   // WiFi Channel (1-13)
#ifdef PLATFORM_REMOTE
    #define RECEIVER_MAC BASE_MAC  
#elif defined(PLATFORM_BASE)
    #define RECEIVER_MAC REMOTE_MAC  
#else
    #error "You must define PLATFORM_REMOTE or PLATFORM_BASE"
#endif
#define LONG_RANGE              true                // Enable Long Range Mode
#define EXTERNAL_ANTENNA        true                // Enable external antenna on Seeedstudio Xiao ESP32 C6 on Pin 14
#define EXTERNAL_ANTENNA_PIN    14                  // External antenna pin that gets set to HIGH


// ***************** UART configuration *****************
#define UART_TX_PIN             16                  // UART TX pin
#define UART_RX_PIN             17                  // UART RX pin
#define UART_PORT_NUM           UART_NUM_1
#define UART_BAUD_RATE          115200
#define UART_DATA_BITS          UART_DATA_8_BITS
#define UART_PARITY             UART_PARITY_DISABLE
#define UART_STOP_BITS          UART_STOP_BITS_1
#define UART_FLOW_CTRL          UART_HW_FLOWCTRL_DISABLE
#define UART_BUFFER_SIZE        1024
#endif // CONFIG_H
