# _Wireless UART bridge using ESP-NOW_
UART bridge between two ESP32. I created this to establish a MAVLink connection between my drone (remote) and the ground station (base).
I'm using a Seeedstudio Xiao ESP32C6 due to its compact size, low cost and the option to use an external antenna.

There are several debug flags, i.e. RSSI. All debug flags are off by default.



## Config
This bridge is intended for establishing a point to point connection between 2 devices. The MAC IDs for the remote/base ESP32 and UART GPIOs are being configured in config.h
Use either #define PLATFORM_BASE or #define PLATFORM_REMOTE accordingly, the then configured MAC adresses will match their peer. PLATFORM_BASE will utilize the USB JTAG port
to receive and send data.

Setting LONG_RANGE will enable the ESPNOW specific long range mode
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html

Supposedly enabling line of sight ranges up to 1km albeit at reduced data rates. Range hasn't been tested yet, the default setting is LONG_RANGE true with a baud rate of 38400.



![IMG_06 11 2024_08 47 25_416 MV](https://github.com/user-attachments/assets/e4cde904-284b-4eaa-abfa-a35d535a0d52)

![Screenshot_20241104_155416](https://github.com/user-attachments/assets/2d80c31d-95c5-4a46-affe-edd422a760b5)


Testing the difference in signal strength between EXTERNAL_ANTENNA true/false to verify that the external antenna is indeed being used
![Screenshot_20241105_193531](https://github.com/user-attachments/assets/fd06231c-6caa-442b-8157-ce145f55d42d)

![Screenshot_20241105_193628](https://github.com/user-attachments/assets/103c5952-54ea-408c-bebd-a86494959c2b)



