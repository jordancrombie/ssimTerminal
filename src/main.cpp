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
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <lvgl.h>
#include <qrcode.h>

#include "pins_config.h"
#include "pmu.h"
#include "display.h"
#include "touch.h"

// =============================================================================
// Configuration
// =============================================================================
#define FIRMWARE_VERSION    "0.3.0"
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

// Timing
static unsigned long lastHeartbeat = 0;
static unsigned long lastLvglTick = 0;
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
void loadStoredConfig();
void saveConfig();
void setupWiFi();
void connectWebSocket();
void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void handleServerMessage(const char *payload);
void sendHeartbeat();
void sendAck(const char *messageId);
void sendPaymentStatus(const char *paymentId, const char *status);
void transitionTo(TerminalState newState);
bool pairWithServer(const char *pairingCode);

// UI screens
void showBootScreen();
void showPairingScreen();
void showConnectingScreen();
void showIdleScreen();
void showQrScreen(const char *qrData, int amount, const char *currency, const char *description);
void showResultScreen(const char *status, const char *message);
void showErrorScreen(const char *message);

// Audio
void playResultSound(const char *status);

// =============================================================================
// LVGL Tick
// =============================================================================
static void lvgl_tick_task(void *arg) {
    while (1) {
        lv_tick_inc(5);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
    Serial.begin(115200);

    // Wait for USB CDC to connect (required for ESP32-S3 USB serial)
    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait) < 3000) {
        delay(10);
    }
    delay(100);  // Extra stability delay

    Serial.println("\n\n========================================");
    Serial.printf("  ssimTerminal v%s\n", FIRMWARE_VERSION);
    Serial.println("========================================");
    Serial.printf("Chip: %s, Rev: %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("CPU: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Flash: %d MB\n", ESP.getFlashChipSize() / 1024 / 1024);
    Serial.printf("PSRAM: %d bytes (%d KB)\n", ESP.getPsramSize(), ESP.getPsramSize() / 1024);
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("----------------------------------------");

    // Initialize PMU first - provides power to display and peripherals
    if (!pmu_init()) {
        Serial.println("WARNING: PMU initialization failed - display may not work!");
    }

    // Initialize LVGL
    Serial.println("Initializing LVGL...");
    lv_init();

    // Create LVGL tick task
    xTaskCreatePinnedToCore(lvgl_tick_task, "lvgl_tick", 2048, NULL, 1, NULL, 0);

    // Initialize display (SH8601)
    if (!display_init()) {
        Serial.println("FATAL: Display initialization failed!");
        while (1) { delay(1000); }
    }

    // Initialize touch (FT3168)
    if (!touch_init()) {
        Serial.println("WARNING: Touch initialization failed - continuing without touch");
    }

    // Initialize NVS for storing credentials
    preferences.begin("ssimterm", false);
    loadStoredConfig();

    // Show boot screen
    showBootScreen();

    // Connect to WiFi first (required for pairing and WebSocket)
    showConnectingScreen();
    setupWiFi();

    if (WiFi.status() != WL_CONNECTED) {
        showErrorScreen("WiFi connection failed");
        transitionTo(TerminalState::ERROR);
        Serial.println("Setup complete (WiFi failed)!");
        return;
    }

    // Check if we have stored credentials
    if (apiKey.isEmpty()) {
        // No API key stored - need to pair
        Serial.println("No API key found, attempting to pair...");

        // For development: use hardcoded pairing code
        // TODO: Replace with UI-based pairing flow
        if (pairWithServer("558067")) {
            Serial.println("Pairing successful, connecting to WebSocket...");
            connectWebSocket();
            transitionTo(TerminalState::IDLE);
        } else {
            Serial.println("Pairing failed!");
            showErrorScreen("Pairing failed\nCheck code and retry");
            transitionTo(TerminalState::ERROR);
        }
    } else {
        Serial.printf("Using stored API key for terminal: %s\n", terminalId.c_str());
        connectWebSocket();
        transitionTo(TerminalState::IDLE);
    }

    Serial.println("Setup complete!");
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
    // Handle LVGL (must be called frequently)
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

    Serial.printf("Config loaded - Server: %s, Terminal: %s, HasKey: %s\n",
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
    Serial.printf("State transition: %d -> %d\n", (int)currentState, (int)newState);
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
    // TODO: Implement proper WiFi provisioning UI
    // For development, use hardcoded credentials
    const char *ssid = "755Avenue_IOT";
    const char *password = "sun0sr0x";

    Serial.printf("Connecting to WiFi: %s\n", ssid);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        lv_timer_handler();  // Keep LVGL responsive
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected! IP: %s, RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
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
    // Development server
    const char* wsHost = "ssim-dev.banksim.ca";
    const int wsPort = 443;

    String wsPath = "/terminal/ws";
    if (!apiKey.isEmpty()) {
        wsPath += "?apiKey=" + apiKey;
    }

    Serial.printf("Connecting to WSS: %s:%d%s\n", wsHost, wsPort, wsPath.c_str());

    // Use SSL for wss://
    webSocket.beginSSL(wsHost, wsPort, wsPath.c_str());
    webSocket.onEvent(handleWebSocketEvent);
    webSocket.setReconnectInterval(RECONNECT_DELAY);

    Serial.println("WebSocket connecting...");
}

// =============================================================================
// Pairing
// =============================================================================
bool pairWithServer(const char *pairingCode) {
    Serial.printf("Pairing with code: %s\n", pairingCode);

    WiFiClientSecure client;
    client.setInsecure();  // Skip certificate validation for dev

    HTTPClient http;
    http.begin(client, "https://ssim-dev.banksim.ca/api/terminal/pair");
    http.addHeader("Content-Type", "application/json");

    // Build request body
    JsonDocument doc;
    doc["pairingCode"] = pairingCode;
    JsonObject deviceInfo = doc["deviceInfo"].to<JsonObject>();
    deviceInfo["model"] = "ESP32-S3-Touch-AMOLED-1.8";
    deviceInfo["firmwareVersion"] = FIRMWARE_VERSION;
    deviceInfo["macAddress"] = WiFi.macAddress();

    String requestBody;
    serializeJson(doc, requestBody);
    Serial.printf("Pairing request: %s\n", requestBody.c_str());

    int httpCode = http.POST(requestBody);
    Serial.printf("Pairing response code: %d\n", httpCode);

    if (httpCode == 200 || httpCode == 201) {
        String response = http.getString();
        Serial.printf("Pairing response: %s\n", response.c_str());

        JsonDocument respDoc;
        DeserializationError error = deserializeJson(respDoc, response);
        if (error) {
            Serial.printf("JSON parse error: %s\n", error.c_str());
            http.end();
            return false;
        }

        if (respDoc["success"] == true) {
            terminalId = respDoc["terminalId"].as<String>();
            apiKey = respDoc["apiKey"].as<String>();

            // Save to NVS
            preferences.putString("terminal_id", terminalId);
            preferences.putString("api_key", apiKey);
            preferences.putString("server_url", "ssim-dev.banksim.ca");

            Serial.printf("Paired successfully! Terminal: %s\n", terminalId.c_str());
            http.end();
            return true;
        } else {
            Serial.printf("Pairing failed: %s\n", respDoc["error"].as<const char*>());
        }
    } else {
        Serial.printf("HTTP error: %s\n", http.errorToString(httpCode).c_str());
        String response = http.getString();
        Serial.printf("Response: %s\n", response.c_str());
    }

    http.end();
    return false;
}

void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket disconnected");
            if (currentState == TerminalState::QR_DISPLAY) {
                // Show warning but don't clear QR yet (per SSIM team guidance)
                // TODO: Show "Connection Lost" overlay
                Serial.println("Connection lost during payment - maintaining QR display");
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
            Serial.printf("WebSocket message: %.*s\n", (int)length, (char *)payload);
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
    const char *messageId = doc["messageId"] | "";

    Serial.printf("Received message type: %s\n", type);

    if (strcmp(type, "payment_request") == 0) {
        // SSIM sends: {"type": "payment_request", "payload": {"paymentId": "...", "qrCodeUrl": "...", "amount": 1500, "currency": "CAD"}}
        JsonObject p = doc["payload"];
        currentPaymentId = p["paymentId"].as<String>();
        currentQrData = p["qrCodeUrl"].as<String>();  // Note: qrCodeUrl not qrData
        currentAmount = p["amount"];
        currentCurrency = p["currency"] | "CAD";
        const char *description = p["orderDescription"] | "";

        Serial.printf("Payment request: %s, Amount: %d %s\n",
                      currentPaymentId.c_str(), currentAmount, currentCurrency.c_str());

        // Calculate local expiry time (5 minute default)
        qrExpiresAt = millis() + (5 * 60 * 1000);

        showQrScreen(currentQrData.c_str(), currentAmount, currentCurrency.c_str(), description);
        transitionTo(TerminalState::QR_DISPLAY);

        // Send status back to SSIM
        sendPaymentStatus(currentPaymentId.c_str(), "displayed");
    }
    else if (strcmp(type, "payment_cancel") == 0) {
        JsonObject p = doc["payload"];
        const char *paymentId = p["paymentId"];
        Serial.printf("Payment cancelled: %s\n", paymentId);

        showResultScreen("cancelled", "Payment Cancelled");
        transitionTo(TerminalState::RESULT);
    }
    else if (strcmp(type, "payment_result") == 0) {
        JsonObject p = doc["payload"];
        const char *status = p["status"];
        const char *message = p["message"] | "";

        showResultScreen(status, message);
        transitionTo(TerminalState::RESULT);
        playResultSound(status);
    }
    else if (strcmp(type, "payment_complete") == 0) {
        // SSIM sends this when payment is finalized
        JsonObject p = doc["payload"];
        const char *paymentId = p["paymentId"];
        const char *status = p["status"];  // "approved", "declined", etc.

        Serial.printf("Payment complete: %s = %s\n", paymentId, status);

        // Show appropriate result screen
        if (strcmp(status, "approved") == 0) {
            showResultScreen("approved", "Payment Successful");
        } else if (strcmp(status, "declined") == 0) {
            showResultScreen("declined", "Payment Declined");
        } else {
            showResultScreen(status, "");
        }
        transitionTo(TerminalState::RESULT);
        playResultSound(status);
    }
    else if (strcmp(type, "config_update") == 0) {
        JsonObject p = doc["payload"];
        // TODO: Handle config updates (heartbeatInterval, etc.)
        Serial.println("Config update received");
    }
    else if (strcmp(type, "pong") == 0) {
        Serial.println("Pong received");
    }
    else if (strcmp(type, "clear") == 0) {
        transitionTo(TerminalState::IDLE);
    }
}

void sendHeartbeat() {
    if (!webSocket.isConnected()) return;

    JsonDocument doc;
    doc["type"] = "heartbeat";
    JsonObject payload = doc["payload"].to<JsonObject>();
    payload["terminalId"] = terminalId;
    payload["firmwareVersion"] = FIRMWARE_VERSION;
    payload["uptime"] = millis() / 1000;
    payload["freeMemory"] = ESP.getFreeHeap();
    payload["wifiRssi"] = WiFi.RSSI();
    payload["ipAddress"] = WiFi.localIP().toString();

    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
    Serial.printf("Heartbeat sent: uptime=%lus, heap=%d, rssi=%d\n",
                  millis() / 1000, ESP.getFreeHeap(), WiFi.RSSI());
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

void sendPaymentStatus(const char *paymentId, const char *status) {
    if (!webSocket.isConnected()) return;

    JsonDocument doc;
    doc["type"] = "payment_status";
    JsonObject payload = doc["payload"].to<JsonObject>();
    payload["paymentId"] = paymentId;
    payload["status"] = status;

    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
    Serial.printf("Sent payment_status: %s = %s\n", paymentId, status);
}

// =============================================================================
// UI Screens (Basic LVGL Implementation)
// =============================================================================

static lv_obj_t *current_screen = nullptr;

/**
 * @brief Switch to a new screen, safely deleting the old one
 * @return The new screen object
 */
static lv_obj_t* switchScreen() {
    lv_obj_t *old_screen = current_screen;
    current_screen = lv_obj_create(NULL);
    lv_scr_load(current_screen);

    // Delete old screen after loading new one
    if (old_screen) {
        lv_obj_del(old_screen);
    }
    return current_screen;
}

void showBootScreen() {
    Serial.println("UI: Boot screen");

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_black(), 0);

    lv_obj_t *label = lv_label_create(current_screen);
    lv_label_set_text(label, "ssimTerminal");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    lv_obj_center(label);

    lv_obj_t *version = lv_label_create(current_screen);
    lv_label_set_text_fmt(version, "v%s", FIRMWARE_VERSION);
    lv_obj_set_style_text_color(version, lv_color_hex(0x888888), 0);
    lv_obj_align(version, LV_ALIGN_CENTER, 0, 40);
}

void showPairingScreen() {
    Serial.println("UI: Pairing screen");

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_black(), 0);

    lv_obj_t *title = lv_label_create(current_screen);
    lv_label_set_text(title, "Setup Required");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t *msg = lv_label_create(current_screen);
    lv_label_set_text(msg, "Configure WiFi and\nSSIM server to continue");
    lv_obj_set_style_text_color(msg, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_align(msg, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(msg);

    // TODO: Add WiFi configuration UI
}

void showConnectingScreen() {
    Serial.println("UI: Connecting screen");

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_black(), 0);

    lv_obj_t *spinner = lv_spinner_create(current_screen, 1000, 60);
    lv_obj_set_size(spinner, 80, 80);
    lv_obj_align(spinner, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *label = lv_label_create(current_screen);
    lv_label_set_text(label, "Connecting...");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 60);
}

void showIdleScreen() {
    Serial.println("UI: Idle screen");

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_black(), 0);

    lv_obj_t *title = lv_label_create(current_screen);
    lv_label_set_text(title, "Ready for Payment");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_center(title);

    lv_obj_t *sub = lv_label_create(current_screen);
    lv_label_set_text(sub, "Waiting for transaction...");
    lv_obj_set_style_text_color(sub, lv_color_hex(0x888888), 0);
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, 40);
}

// Static buffer for QR code canvas (allocated once to avoid fragmentation)
static lv_color_t *qr_canvas_buf = nullptr;
static const int QR_CANVAS_SIZE = 240;  // Canvas size in pixels

void showQrScreen(const char *qrData, int amount, const char *currency, const char *description) {
    Serial.printf("UI: QR screen - Amount: %d %s, URL: %s\n", amount, currency, qrData);

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_black(), 0);

    // Title
    lv_obj_t *title = lv_label_create(current_screen);
    lv_label_set_text(title, "Scan to Pay");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Generate QR Code using ricmoo/QRCode library
    // Version 6 = 41x41 modules, can encode ~84 alphanumeric chars
    QRCode qrcode;
    uint8_t qrcodeData[qrcode_getBufferSize(6)];

    int qrResult = qrcode_initText(&qrcode, qrcodeData, 6, ECC_MEDIUM, qrData);

    if (qrResult != 0) {
        Serial.printf("QR generation failed: %d\n", qrResult);
        // Show error message instead
        lv_obj_t *error = lv_label_create(current_screen);
        lv_label_set_text(error, "QR Code Error");
        lv_obj_set_style_text_color(error, lv_color_hex(0xFF6666), 0);
        lv_obj_center(error);
        return;
    }

    Serial.printf("QR generated: %dx%d modules\n", qrcode.size, qrcode.size);

    // Allocate canvas buffer if not already done
    if (qr_canvas_buf == nullptr) {
        qr_canvas_buf = (lv_color_t *)heap_caps_malloc(
            QR_CANVAS_SIZE * QR_CANVAS_SIZE * sizeof(lv_color_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        );
        if (qr_canvas_buf == nullptr) {
            Serial.println("Failed to allocate QR canvas buffer!");
            return;
        }
    }

    // Create canvas for QR code
    lv_obj_t *canvas = lv_canvas_create(current_screen);
    lv_canvas_set_buffer(canvas, qr_canvas_buf, QR_CANVAS_SIZE, QR_CANVAS_SIZE, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(canvas, LV_ALIGN_CENTER, 0, -15);

    // Fill with white background
    lv_canvas_fill_bg(canvas, lv_color_white(), LV_OPA_COVER);

    // Calculate module size to fit in canvas with margin
    int margin = 8;  // White border around QR
    int availableSize = QR_CANVAS_SIZE - (margin * 2);
    int moduleSize = availableSize / qrcode.size;
    int qrPixelSize = moduleSize * qrcode.size;
    int offsetX = (QR_CANVAS_SIZE - qrPixelSize) / 2;
    int offsetY = (QR_CANVAS_SIZE - qrPixelSize) / 2;

    // Draw QR code modules
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = lv_color_black();
    rect_dsc.bg_opa = LV_OPA_COVER;

    for (int y = 0; y < qrcode.size; y++) {
        for (int x = 0; x < qrcode.size; x++) {
            if (qrcode_getModule(&qrcode, x, y)) {
                // Draw black module
                lv_area_t area;
                area.x1 = offsetX + (x * moduleSize);
                area.y1 = offsetY + (y * moduleSize);
                area.x2 = area.x1 + moduleSize - 1;
                area.y2 = area.y1 + moduleSize - 1;

                // Fill the module area with black pixels
                for (int py = area.y1; py <= area.y2; py++) {
                    for (int px = area.x1; px <= area.x2; px++) {
                        lv_canvas_set_px_color(canvas, px, py, lv_color_black());
                    }
                }
            }
        }
    }

    // Amount display
    char amount_str[32];
    snprintf(amount_str, sizeof(amount_str), "$%.2f %s", amount / 100.0f, currency);
    lv_obj_t *amount_label = lv_label_create(current_screen);
    lv_label_set_text(amount_label, amount_str);
    lv_obj_set_style_text_color(amount_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(amount_label, &lv_font_montserrat_32, 0);
    lv_obj_align(amount_label, LV_ALIGN_BOTTOM_MID, 0, -60);

    // Description (if provided)
    if (description && strlen(description) > 0) {
        lv_obj_t *desc_label = lv_label_create(current_screen);
        lv_label_set_text(desc_label, description);
        lv_obj_set_style_text_color(desc_label, lv_color_hex(0xAAAAAA), 0);
        lv_obj_align(desc_label, LV_ALIGN_BOTTOM_MID, 0, -30);
    }
}

void showResultScreen(const char *status, const char *message) {
    Serial.printf("UI: Result screen - Status: %s\n", status);

    switchScreen();

    lv_color_t bg_color;
    const char *icon_text;
    const char *status_text;

    if (strcmp(status, "approved") == 0) {
        bg_color = lv_color_hex(0x1B5E20);  // Dark green
        icon_text = LV_SYMBOL_OK;
        status_text = "Payment Approved";
    } else if (strcmp(status, "declined") == 0) {
        bg_color = lv_color_hex(0xB71C1C);  // Dark red
        icon_text = LV_SYMBOL_CLOSE;
        status_text = "Payment Declined";
    } else {
        bg_color = lv_color_hex(0xE65100);  // Dark orange
        icon_text = LV_SYMBOL_WARNING;
        status_text = "Payment Error";
    }

    lv_obj_set_style_bg_color(current_screen, bg_color, 0);

    // Icon
    lv_obj_t *icon = lv_label_create(current_screen);
    lv_label_set_text(icon, icon_text);
    lv_obj_set_style_text_color(icon, lv_color_white(), 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_48, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -40);

    // Status text
    lv_obj_t *status_label = lv_label_create(current_screen);
    lv_label_set_text(status_label, status_text);
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_24, 0);
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 30);

    // Message
    if (strlen(message) > 0) {
        lv_obj_t *msg_label = lv_label_create(current_screen);
        lv_label_set_text(msg_label, message);
        lv_obj_set_style_text_color(msg_label, lv_color_hex(0xCCCCCC), 0);
        lv_obj_align(msg_label, LV_ALIGN_CENTER, 0, 70);
    }
}

void showErrorScreen(const char *message) {
    Serial.printf("UI: Error screen - %s\n", message);

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_hex(0x1A1A1A), 0);

    lv_obj_t *icon = lv_label_create(current_screen);
    lv_label_set_text(icon, LV_SYMBOL_WARNING);
    lv_obj_set_style_text_color(icon, lv_color_hex(0xFF9800), 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_48, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t *label = lv_label_create(current_screen);
    lv_label_set_text(label, message);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 30);
}

// =============================================================================
// Audio Feedback
// =============================================================================
void playResultSound(const char *status) {
    // TODO: Implement audio feedback using ES8311 codec
    Serial.printf("Audio: Would play %s sound\n", status);
}
