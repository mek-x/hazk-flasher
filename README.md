# HAZK-03 Flasher (ESP32)

This project turns an ESP32 into a wireless firmware flasher and serial bridge for STM32 microcontrollers. It utilizes the built-in STM32 system bootloader (UART) to flash firmware uploaded via a web interface.

## Features

*   **Web-based Flashing:** Upload `.bin` files directly from your browser to the STM32.
*   **Wireless Serial Bridge:** When not flashing, the ESP32 acts as a transparent UART bridge (USB <-> STM32), allowing you to view debug logs via the ESP32's USB port.
*   **Automatic Bootloader Entry:** Automatically toggles `BOOT0` and `NRST` pins to enter bootloader mode during flashing.
*   **Progress Feedback:** Real-time upload progress bar and status LED indicators.

## Hardware Connections

The default pin configuration is set up for the **Waveshare ESP32 One**, but can be adapted for other ESP32 boards in `src/main.cpp`.

| ESP32 Pin | STM32 Pin | Function |
| :--- | :--- | :--- |
| IO 13 | RX | UART TX (ESP32 sends to STM32) |
| IO 14 | TX | UART RX (ESP32 receives from STM32) |
| IO 23 | BOOT0 | Bootloader Control |
| IO 18 | NRST | Reset Control |
| IO 21 | LED | Status LED |
| GND | GND | Common Ground |

## Setup & Installation

### 1. Prerequisites
*   Visual Studio Code
*   PlatformIO Extension

### 2. Configuration
For security, WiFi credentials are not stored in the source code. You must create a `credentials.ini` file in the project root (this file is ignored by git).

Create a file named `credentials.ini` in the root directory:
```ini
[env]
build_flags =
    -D WIFI_SSID='"YourSSID"'
    -D WIFI_PASS='"YourPassword"'
```

### 3. Build & Upload
1.  Open the project in VS Code with PlatformIO.
2.  Connect your ESP32 via USB.
3.  Click the **PlatformIO: Upload** button (right arrow icon) in the bottom status bar.

## Usage

1.  **Power Up:** Connect the ESP32 and the STM32 target.
2.  **Connect:** Open the Serial Monitor (Baud 115200) to see the assigned IP address.
3.  **Web Interface:** Navigate to `http://<ESP32-IP>/` in your web browser.
4.  **Identify:** Click "Identify Target" to verify wiring and chip detection.
5.  **Flash:** Select your STM32 firmware (`.bin`) and click "Flash Firmware".

## Development

This project includes a Dev Container configuration for VS Code, allowing you to develop inside a consistent Docker environment with all dependencies pre-installed.

## License
MIT
