/**
 * @file main.cpp
 * @brief ssimTerminal - ESP32-S3 Payment Terminal Firmware
 *
 * A WebSocket-connected payment terminal that displays QR codes for mobile
 * wallet payments and shows transaction results.
 *
 * Hardware: Waveshare ESP32-S3-Touch-AMOLED-1.8
 * Display: 368x448 AMOLED with SH8601 driver (QSPI)
 * Touch: FT3168 capacitive touch controller (I2C)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <lvgl.h>
#include "pins_config.h"

// =============================================================================
// Configuration
// =============================================================================
#define FIRMWARE_VERSION    "0.1.0"
#define HEARTBEAT_INTERVAL  30000   // 30 seconds
#define RECONNECT_DELAY     5000    // 5 seconds
#define RESULT_DISPLAY_MS   3000    // 3 seconds to show result

// =============================================================================
// Terminal States
// =============================================================================
enum class TerminalState {
    BOOT,           // Hardware initialization
    PAIRING,        // First-time setup, entering server URL and pairing code
    CONNECTING,     // Connecting to SSIM WebSocket
    IDLE,           // Ready for payment, showing store branding
    QR_DISPLAY,     // Showing QR code with amount and countdown
    RESULT,         // Payment outcome display
    ERROR           // Error state (connection lost, etc.)
};

// =============================================================================
// Globals
// =============================================================================
static TerminalState currentState = TerminalState::BOOT;
static WebSocketsClient webSocket;
static Preferences preferences;

// Display buffer for LVGL
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;

// Timing
static unsigned long lastHeartbeat = 0;
static unsigned long resultStartTime = 0;
static unsigned long qrExpiresAt = 0;

// Current payment info
static String currentPaymentId;
static String currentQrData;
static int currentAmount = 0;
static String currentCurrency;

// Terminal config (stored in NVS)
static String ssimServerUrl;
static String apiKey;
static String terminalId;

// =============================================================================
// Forward Declarations
// =============================================================================
void setupDisplay();
void setupTouch();
void setupWiFi();
void connectWebSocket();
void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void handleServerMessage(const char *payload);
void sendHeartbeat();
void transitionTo(TerminalState newState);

// UI screens
void showBootScreen();
void showPairingScreen();
void showConnectingScreen();
void showIdleScreen();
void showQrScreen(const char *qrData, int amount, const char *currency, const char *description);
void showResultScreen(const char *status, const char *message);
void showErrorScreen(const char *message);

// LVGL callbacks
void lvgl_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void lvgl_touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data);

// =============================================================================
// Setup
// =============================================================================
void setup() {
    Serial.begin(115200);
    Serial.printf("\n\n=== ssimTerminal v%s ===\n", FIRMWARE_VERSION);
    Serial.printf("Chip: %s, Rev: %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("Flash: %d MB, PSRAM: %d MB\n",
                  ESP.getFlashChipSize() / 1024 / 1024,
                  ESP.getPsramSize() / 1024 / 1024);

    // Initialize NVS for storing credentials
    preferences.begin("ssimterm", false);
    loadStoredConfig();

    // Initialize LVGL
    lv_init();
    setupDisplay();
    setupTouch();

    showBootScreen();

    // Check if we have stored credentials
    if (apiKey.isEmpty()) {
        transitionTo(TerminalState::PAIRING);
    } else {
        transitionTo(TerminalState::CONNECTING);
    }
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
    // Handle LVGL
    lv_timer_handler();

    // Handle WebSocket
    if (currentState != TerminalState::BOOT && currentState != TerminalState::PAIRING) {
        webSocket.loop();
    }

    // State-specific logic
    switch (currentState) {
        case TerminalState::IDLE:
            // Send periodic heartbeat
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                sendHeartbeat();
                lastHeartbeat = millis();
            }
            break;

        case TerminalState::QR_DISPLAY:
            // Check for local timeout
            if (qrExpiresAt > 0 && millis() >= qrExpiresAt) {
                Serial.println("QR code expired locally");
                showResultScreen("expired", "Payment expired");
                transitionTo(TerminalState::RESULT);
            }
            break;

        case TerminalState::RESULT:
            // Auto-return to idle after showing result
            if (millis() - resultStartTime >= RESULT_DISPLAY_MS) {
                transitionTo(TerminalState::IDLE);
            }
            break;

        default:
            break;
    }

    delay(5);  // Small delay to prevent watchdog issues
}

// =============================================================================
// Configuration Loading
// =============================================================================
void loadStoredConfig() {
    ssimServerUrl = preferences.getString("server_url", "");
    apiKey = preferences.getString("api_key", "");
    terminalId = preferences.getString("terminal_id", "");

    Serial.printf("Loaded config - Server: %s, Terminal: %s, HasKey: %s\n",
                  ssimServerUrl.isEmpty() ? "(none)" : ssimServerUrl.c_str(),
                  terminalId.isEmpty() ? "(none)" : terminalId.c_str(),
                  apiKey.isEmpty() ? "no" : "yes");
}

void saveConfig() {
    preferences.putString("server_url", ssimServerUrl);
    preferences.putString("api_key", apiKey);
    preferences.putString("terminal_id", terminalId);
    Serial.println("Configuration saved to NVS");
}

void clearConfig() {
    preferences.clear();
    ssimServerUrl = "";
    apiKey = "";
    terminalId = "";
    Serial.println("Configuration cleared");
}

// =============================================================================
// State Transitions
// =============================================================================
void transitionTo(TerminalState newState) {
    Serial.printf("State: %d -> %d\n", (int)currentState, (int)newState);
    currentState = newState;

    switch (newState) {
        case TerminalState::BOOT:
            showBootScreen();
            break;

        case TerminalState::PAIRING:
            showPairingScreen();
            break;

        case TerminalState::CONNECTING:
            showConnectingScreen();
            setupWiFi();
            connectWebSocket();
            break;

        case TerminalState::IDLE:
            showIdleScreen();
            currentPaymentId = "";
            currentQrData = "";
            currentAmount = 0;
            break;

        case TerminalState::QR_DISPLAY:
            // QR screen is shown by handleServerMessage
            break;

        case TerminalState::RESULT:
            resultStartTime = millis();
            break;

        case TerminalState::ERROR:
            // Error screen is shown directly
            break;
    }
}

// =============================================================================
// WiFi Setup
// =============================================================================
void setupWiFi() {
    // TODO: Implement WiFi provisioning UI
    // For now, use hardcoded credentials for testing
    const char *ssid = "YOUR_WIFI_SSID";
    const char *password = "YOUR_WIFI_PASSWORD";

    Serial.printf("Connecting to WiFi: %s\n", ssid);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connection failed!");
        showErrorScreen("WiFi connection failed");
        transitionTo(TerminalState::ERROR);
    }
}

// =============================================================================
// WebSocket
// =============================================================================
void connectWebSocket() {
    if (ssimServerUrl.isEmpty()) {
        Serial.println("No server URL configured");
        return;
    }

    // Parse URL and connect
    // Expected format: wss://ssim.example.com/terminal/ws?apiKey=xxx
    String wsUrl = ssimServerUrl + "/terminal/ws?apiKey=" + apiKey;

    // For development, using ws:// (insecure)
    // TODO: Switch to wss:// for production
    webSocket.begin("localhost", 3000, "/terminal/ws?apiKey=" + apiKey);
    webSocket.onEvent(handleWebSocketEvent);
    webSocket.setReconnectInterval(RECONNECT_DELAY);

    Serial.println("WebSocket connecting...");
}

void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket disconnected");
            if (currentState == TerminalState::QR_DISPLAY) {
                // Show warning but don't clear QR yet (per SSIM team guidance)
                // TODO: Show "Connection Lost" overlay
            } else if (currentState != TerminalState::PAIRING) {
                showErrorScreen("Connection lost\nReconnecting...");
            }
            break;

        case WStype_CONNECTED:
            Serial.printf("WebSocket connected to: %s\n", (char *)payload);
            transitionTo(TerminalState::IDLE);
            sendHeartbeat();
            break;

        case WStype_TEXT:
            Serial.printf("WebSocket message: %s\n", (char *)payload);
            handleServerMessage((char *)payload);
            break;

        case WStype_ERROR:
            Serial.println("WebSocket error");
            break;

        default:
            break;
    }
}

void handleServerMessage(const char *payload) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.printf("JSON parse error: %s\n", error.c_str());
        return;
    }

    const char *type = doc["type"];

    if (strcmp(type, "display_qr") == 0) {
        // Display QR code for payment
        JsonObject p = doc["payload"];
        currentPaymentId = p["paymentId"].as<String>();
        currentQrData = p["qrData"].as<String>();
        currentAmount = p["amount"];
        currentCurrency = p["currency"].as<String>();
        const char *description = p["orderDescription"] | "";
        const char *expiresAt = p["expiresAt"] | "";

        // Calculate local expiry time (simplified - should parse ISO timestamp)
        // For now, assume 5 minute timeout
        qrExpiresAt = millis() + (5 * 60 * 1000);

        showQrScreen(currentQrData.c_str(), currentAmount, currentCurrency.c_str(), description);
        transitionTo(TerminalState::QR_DISPLAY);

        // Send acknowledgment
        sendAck(doc["messageId"] | "");
    }
    else if (strcmp(type, "payment_result") == 0) {
        // Payment result
        JsonObject p = doc["payload"];
        const char *status = p["status"];
        const char *message = p["message"] | "";

        showResultScreen(status, message);
        transitionTo(TerminalState::RESULT);

        // Play sound feedback
        playResultSound(status);

        sendAck(doc["messageId"] | "");
    }
    else if (strcmp(type, "clear") == 0) {
        // Clear display and return to idle
        transitionTo(TerminalState::IDLE);
        sendAck(doc["messageId"] | "");
    }
    else if (strcmp(type, "config") == 0) {
        // Configuration update
        JsonObject p = doc["payload"];
        // TODO: Handle config updates
        sendAck(doc["messageId"] | "");
    }
}

void sendHeartbeat() {
    if (!webSocket.isConnected()) return;

    JsonDocument doc;
    doc["type"] = "heartbeat";
    JsonObject payload = doc["payload"].to<JsonObject>();
    payload["uptime"] = millis() / 1000;
    payload["freeMemory"] = ESP.getFreeHeap();
    payload["wifiRssi"] = WiFi.RSSI();

    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
}

void sendAck(const char *messageId) {
    if (!webSocket.isConnected() || strlen(messageId) == 0) return;

    JsonDocument doc;
    doc["type"] = "ack";
    JsonObject payload = doc["payload"].to<JsonObject>();
    payload["messageId"] = messageId;
    payload["success"] = true;

    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
}

void sendStatus() {
    if (!webSocket.isConnected()) return;

    JsonDocument doc;
    doc["type"] = "status";
    JsonObject payload = doc["payload"].to<JsonObject>();

    const char *screenName;
    switch (currentState) {
        case TerminalState::IDLE:       screenName = "idle"; break;
        case TerminalState::QR_DISPLAY: screenName = "qr_display"; break;
        case TerminalState::RESULT:     screenName = "result"; break;
        default:                        screenName = "other"; break;
    }
    payload["currentScreen"] = screenName;

    if (!currentPaymentId.isEmpty()) {
        payload["paymentId"] = currentPaymentId;
    }

    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
}

// =============================================================================
// Display Setup (Placeholder - needs SH8601 driver)
// =============================================================================
void setupDisplay() {
    // Allocate draw buffers in PSRAM
    size_t bufSize = LCD_WIDTH * 40;  // 40 lines at a time
    buf1 = (lv_color_t *)ps_malloc(bufSize * sizeof(lv_color_t));
    buf2 = (lv_color_t *)ps_malloc(bufSize * sizeof(lv_color_t));

    if (!buf1 || !buf2) {
        Serial.println("ERROR: Failed to allocate display buffers!");
        return;
    }

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, bufSize);

    // Initialize display driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    Serial.println("Display initialized");

    // TODO: Initialize SH8601 QSPI display driver
    // This requires the actual SH8601 driver library from Waveshare
}

void lvgl_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    // TODO: Implement SH8601 flush
    // This function sends pixel data to the display

    // For now, just signal completion
    lv_disp_flush_ready(disp);
}

// =============================================================================
// Touch Setup (Placeholder - needs FT3168 driver)
// =============================================================================
void setupTouch() {
    // TODO: Initialize I2C and FT3168 touch controller

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    Serial.println("Touch initialized");
}

void lvgl_touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data) {
    // TODO: Read from FT3168 touch controller
    data->state = LV_INDEV_STATE_REL;
    data->point.x = 0;
    data->point.y = 0;
}

// =============================================================================
// UI Screens (Placeholders - need LVGL implementation)
// =============================================================================
void showBootScreen() {
    Serial.println("UI: Boot screen");
    // TODO: Show logo and "Starting..." message
}

void showPairingScreen() {
    Serial.println("UI: Pairing screen");
    // TODO: Show pairing code entry UI
}

void showConnectingScreen() {
    Serial.println("UI: Connecting screen");
    // TODO: Show spinner and "Connecting..." message
}

void showIdleScreen() {
    Serial.println("UI: Idle screen");
    // TODO: Show store branding and "Ready for payment" message
}

void showQrScreen(const char *qrData, int amount, const char *currency, const char *description) {
    Serial.printf("UI: QR screen - Amount: %d %s\n", amount, currency);
    // TODO: Generate and display QR code with amount
}

void showResultScreen(const char *status, const char *message) {
    Serial.printf("UI: Result screen - Status: %s, Message: %s\n", status, message);
    // TODO: Show checkmark/X/warning icon with message
}

void showErrorScreen(const char *message) {
    Serial.printf("UI: Error screen - %s\n", message);
    // TODO: Show error icon and message
}

// =============================================================================
// Audio Feedback (Placeholder - needs ES8311 driver)
// =============================================================================
void playResultSound(const char *status) {
    Serial.printf("Audio: Playing %s sound\n", status);
    // TODO: Implement audio feedback using ES8311 codec
    // - "approved": Pleasant chime/ding
    // - "declined": Two low tones
    // - "failed"/"error": Alert tone
}
