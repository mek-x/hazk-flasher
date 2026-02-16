#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

// == USER CONFIGURATION ==
// WiFi credentials are now defined in `credentials.ini`
// and passed to the compiler as build flags.
#ifndef WIFI_SSID
  #error "WIFI_SSID not defined! Please create a credentials.ini file with your WiFi SSID."
#endif
#ifndef WIFI_PASS
  #error "WIFI_PASS not defined! Please create a credentials.ini file with your WiFi Password."
#endif

// == PIN DEFINITIONS (Waveshare ESP32 One) ==
// Header Pin 19 -> IO13 (Connect to STM32 RX)
#define STM_TX_PIN 13
// Header Pin 23 -> IO14 (Connect to STM32 TX)
#define STM_RX_PIN 14
// Header Pin 5  -> IO23 (Connect to STM32 BOOT0)
#define PIN_BOOT0  23
// Header Pin 3  -> IO18 (Connect to STM32 NRST)
#define PIN_RST    18
// Status LED
#define PIN_LED    21

// == BAUD RATES ==
#define DEBUG_BAUD      115200 // USB Serial
#define BOOTLOADER_BAUD 57600  // Bootloader Sync (Stable)
#define BRIDGE_BAUD     115200 // Normal Operation

WebServer server(80);

// STM32 Protocol Constants (AN3155)
#define STM32_ACK   0x79
#define STM32_NACK  0x1F
#define CMD_INIT    0x7F
#define CMD_GETID   0x02
#define CMD_ERASE   0x43
#define CMD_WRITE   0x31

// Global State
bool flashingMode = false;
bool flashSuccess = false;
uint32_t flashAddress = 0x08000000;
uint8_t stmBuffer[256];
uint16_t stmBufferIndex = 0;
size_t flashBytesReceived = 0;

// ==========================================
// STM32 HARDWARE CONTROL
// ==========================================

void resetSTM32(bool enterBootloader) {
    Serial.printf("Target Reset (Mode: %s)...\n", enterBootloader ? "BOOTLOADER" : "RUN");
    digitalWrite(PIN_BOOT0, enterBootloader ? HIGH : LOW);
    delay(50);
    digitalWrite(PIN_RST, LOW);
    delay(100);
    digitalWrite(PIN_RST, HIGH);
    delay(enterBootloader ? 500 : 200);
}

// ==========================================
// STM32 PROTOCOL
// ==========================================

void sendByte(uint8_t b) {
    Serial1.write(b);
}

bool waitForACK() {
    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (b == STM32_ACK) return true;
            if (b == STM32_NACK) return false;
        }
    }
    return false;
}

bool stm32_sync() {
    // 1. Configure Serial1 for Bootloader (8E1)
    Serial1.end();
    Serial1.begin(BOOTLOADER_BAUD, SERIAL_8E1, STM_RX_PIN, STM_TX_PIN);
    while(Serial1.available()) Serial1.read(); // Clear junk

    // 2. Send Init Byte
    sendByte(CMD_INIT);
    return waitForACK();
}

int stm32_get_id() {
    sendByte(CMD_GETID);
    sendByte(0xFF ^ CMD_GETID);

    if (!waitForACK()) return -1;

    unsigned long start = millis();
    while (!Serial1.available()) { if(millis()-start > 500) return -2; }
    uint8_t n = Serial1.read(); // Number of bytes - 1

    uint16_t pid = 0;
    for (int i=0; i <= n; i++) {
        while (!Serial1.available()) { if(millis()-start > 500) return -3; }
        uint8_t val = Serial1.read();
        pid = (pid << 8) | val;
    }

    if (!waitForACK()) return -4;
    return pid;
}

bool stm32_erase() {
    sendByte(CMD_ERASE);
    sendByte(0xFF ^ CMD_ERASE);
    if (!waitForACK()) return false;
    sendByte(0xFF);
    sendByte(0x00);
    return waitForACK();
}

bool stm32_write_chunk(uint8_t* data, uint16_t len) {
    sendByte(CMD_WRITE);
    sendByte(0xFF ^ CMD_WRITE);
    if (!waitForACK()) return false;

    uint8_t checksum = 0;
    sendByte((flashAddress >> 24) & 0xFF); checksum ^= ((flashAddress >> 24) & 0xFF);
    sendByte((flashAddress >> 16) & 0xFF); checksum ^= ((flashAddress >> 16) & 0xFF);
    sendByte((flashAddress >> 8)  & 0xFF); checksum ^= ((flashAddress >> 8)  & 0xFF);
    sendByte((flashAddress >> 0)  & 0xFF); checksum ^= ((flashAddress >> 0)  & 0xFF);
    sendByte(checksum);
    if (!waitForACK()) return false;

    sendByte(len - 1);
    checksum = (len - 1);
    for (int i = 0; i < len; i++) {
        sendByte(data[i]);
        checksum ^= data[i];
    }
    sendByte(checksum);

    bool result = waitForACK();
    if (result) flashAddress += len;
    return result;
}

// ==========================================
// WEB SERVER HANDLERS
// ==========================================

const char* html_page = R"(
<html>
<head>
<title>HAZK-03 Flasher</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body { font-family: sans-serif; padding: 20px; max-width: 600px; margin: auto; background: #fafafa; }
    h1 { text-align: center; color: #333; }
    .card { background: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    .btn { display: block; width: 100%; padding: 12px; margin-top: 10px; border: none; cursor: pointer; font-size: 16px; border-radius: 4px; font-weight: bold; }
    .btn-check { background: #28a745; color: white; }
    .btn-flash { background: #007bff; color: white; }
    #status { margin-top: 15px; padding: 10px; background: #eee; border-radius: 4px; font-family: monospace; white-space: pre-wrap; min-height: 40px; }
    #progress { width: 100%; background-color: #ddd; border-radius: 4px; margin-top: 10px; display: none; }
    #bar { width: 0%; height: 20px; background-color: #007bff; border-radius: 4px; text-align: center; line-height: 20px; color: white; font-size: 12px; }
</style>
<script>
    function checkConnection() {
        document.getElementById('status').innerText = "Checking connection...";
        fetch('/identify').then(r => r.text()).then(t => {
            document.getElementById('status').innerText = t;
        });
    }
    function uploadFirmware() {
        var input = document.getElementById('file_input');
        if(input.files.length === 0){ alert("Select a file first!"); return; }
        var file = input.files[0];
        var formData = new FormData();
        formData.append("update", file);
        var xhr = new XMLHttpRequest();
        document.getElementById('progress').style.display = 'block';
        document.getElementById('status').innerText = "Uploading...";

        xhr.upload.addEventListener("progress", function(e) {
            if (e.lengthComputable) {
                var percent = Math.round((e.loaded / e.total) * 100);
                document.getElementById('bar').style.width = percent + "%";
                document.getElementById('bar').innerText = percent + "%";
            }
        }, false);

        xhr.onload = function() {
            document.getElementById('status').innerText = xhr.responseText;
            var color = (xhr.status == 200 && xhr.responseText.indexOf("OK") != -1) ? "#28a745" : "#dc3545";
            document.getElementById('bar').style.backgroundColor = color;
        };
        xhr.open("POST", "/update");
        xhr.send(formData);
    }
</script>
</head>
<body>
    <h1>HAZK-03 Flasher</h1>

    <div class="card">
        <h3>1. Diagnostics</h3>
        <p>Verify wiring and bootloader mode.</p>
        <button class="btn btn-check" onclick=checkConnection()>Identify Target</button>
        <div id="status">Ready.</div>
    </div>

    <div class="card">
        <h3>2. Firmware Update</h3>
        <p>Upload 'firmware.bin' to flash.</p>
        <input type='file' id='file_input' name='update' style="margin-bottom: 10px;">
        <button class='btn btn-flash' onclick=uploadFirmware()>Flash Firmware</button>
        <div id="progress"><div id="bar">0%</div></div>
    </div>
</body>
</html>
)";

void handleRoot() {
    server.send(200, "text/html", html_page);
}

void handleIdentify() {
    flashingMode = true;
    resetSTM32(true);

    if (!stm32_sync()) {
        server.send(200, "text/plain", "Error: Sync Failed.\n- Check wiring (TX/RX flipped?)\n- Check Power");
    } else {
        int pid = stm32_get_id();
        String msg = "Success! Chip Detected.\nPID: 0x" + String(pid, HEX);
        if (pid == 0x418) msg += " (STM32F105/107 Connectivity Line)";
        server.send(200, "text/plain", msg);
    }

    resetSTM32(false);
    Serial1.end();
    Serial1.begin(BRIDGE_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);
    flashingMode = false;
}

void handleUpdateResult() {
    Serial.println(flashSuccess ? "FLASH COMPLETE" : "FLASH FAILED");

    resetSTM32(false);
    Serial1.end();
    Serial1.begin(BRIDGE_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);
    flashingMode = false;
    server.send(200, "text/plain", flashSuccess ? "OK: Flashed & Rebooted" : "FAIL: Upload Error");
}

void handleUpdateUpload() {
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        flashingMode = true;
        flashSuccess = true;
        flashAddress = 0x08000000;
        stmBufferIndex = 0;
        flashBytesReceived = 0;

        Serial.println(">> STARTING FLASH");
        resetSTM32(true);
        if (!stm32_sync()) {
            Serial.println("SYNC FAIL");
            flashSuccess = false;
            return;
        }
        Serial.println("SYNC OK. Erasing...");
        if (!stm32_erase()) {
            Serial.println("ERASE FAIL");
            flashSuccess = false;
            return;
        }
        Serial.println("ERASE OK. Writing...");

    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (!flashSuccess) return;

        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        flashBytesReceived += upload.currentSize;
        if (upload.totalSize > 0) {
            Serial.printf("Progress: %u%%\r", (flashBytesReceived * 100) / upload.totalSize);
        }

        for (size_t i = 0; i < upload.currentSize; i++) {
            stmBuffer[stmBufferIndex++] = upload.buf[i];
            if (stmBufferIndex == 256) {
                if (!stm32_write_chunk(stmBuffer, 256)) flashSuccess = false;
                stmBufferIndex = 0;
            }
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        digitalWrite(PIN_LED, HIGH);
        if (flashSuccess && stmBufferIndex > 0) {
            if (!stm32_write_chunk(stmBuffer, stmBufferIndex)) flashSuccess = false;
        }
        Serial.printf("\nDONE. Size: %u\n", upload.totalSize);
    }
}

// ==========================================
// MAIN LOOP
// ==========================================

void setup() {
    Serial.begin(DEBUG_BAUD);
    Serial1.begin(BRIDGE_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);

    pinMode(PIN_BOOT0, OUTPUT);
    pinMode(PIN_RST, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_RST, HIGH);
    digitalWrite(PIN_LED, HIGH);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nReady!");
    Serial.print("Web Interface: http://"); Serial.println(WiFi.localIP());

    // == OTA SETUP ==
    ArduinoOTA.setHostname("hazk-flasher");
    ArduinoOTA.onStart([]() {
        flashingMode = true; // Stop bridge during update
        Serial.println("OTA Start");
    });
    ArduinoOTA.onEnd([]() {
        flashingMode = false;
        Serial.println("\nOTA End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();

    server.on("/", handleRoot);
    server.on("/identify", handleIdentify);
    server.on("/update", HTTP_POST, handleUpdateResult, handleUpdateUpload);
    server.begin();
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
    if (!flashingMode) {
        // Transparent Bridge
        if (Serial.available()) Serial1.write(Serial.read());
        if (Serial1.available()) Serial.write(Serial1.read());
    }
}
