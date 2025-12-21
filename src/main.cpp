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
#include <Wire.h>
#include <driver/i2s.h>

#include "pins_config.h"
#include "pmu.h"
#include "display.h"
#include "touch.h"

// =============================================================================
// Configuration
// =============================================================================
#define FIRMWARE_VERSION    "0.5.0"
#define HEARTBEAT_INTERVAL  30000   // 30 seconds
#define RECONNECT_DELAY     5000    // 5 seconds
#define RESULT_DISPLAY_MS   3000    // 3 seconds to show result

// =============================================================================
// Terminal States
// =============================================================================
enum class TerminalState {
    BOOT,           // Hardware initialization
    WIFI_SETUP,     // WiFi network selection
    WIFI_PASSWORD,  // WiFi password entry (separate state to prevent accidental navigation)
    PAIRING,        // First-time setup, entering server URL and pairing code
    CONNECTING,     // Connecting to SSIM WebSocket
    IDLE,           // Ready for payment, showing store branding
    SETTINGS,       // Settings menu
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
static String storeName;

// Settings (stored in NVS)
static int displayBrightness = 255;    // 0-255
static bool soundEnabled = true;

// WiFi provisioning state
static String wifiNetworks[20];       // List of scanned network SSIDs
static int wifiNetworkCount = 0;      // Number of networks found
static int selectedNetworkIndex = -1; // Currently selected network
static String selectedSSID;           // SSID being configured
static String enteredPassword;        // Password being entered

// Stored WiFi credentials
static String storedWifiSSID;
static String storedWifiPassword;

// Environment configuration
enum class Environment {
    DEVELOPMENT,
    PRODUCTION
};

static Environment currentEnvironment = Environment::DEVELOPMENT;

// Environment-specific server configuration
struct ServerConfig {
    const char* wsHost;
    int wsPort;
    const char* apiHost;
    const char* displayName;
};

static const ServerConfig DEV_CONFIG = {
    "ssim-dev.banksim.ca",
    443,
    "https://ssim-dev.banksim.ca",
    "Development"
};

static const ServerConfig PROD_CONFIG = {
    "store.regalmoose.ca",
    443,
    "https://store.regalmoose.ca",
    "Production"
};

static const ServerConfig* getServerConfig() {
    return (currentEnvironment == Environment::PRODUCTION) ? &PROD_CONFIG : &DEV_CONFIG;
}

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
void showWifiSetupScreen();
void showWifiPasswordScreen();
void showPairingScreen();
void showConnectingScreen();
void showIdleScreen();
void showQrScreen(const char *qrData, int amount, const char *currency, const char *description);
void showResultScreen(const char *status, const char *message);
void showErrorScreen(const char *message);

// WiFi provisioning
void scanWifiNetworks();
void connectToWifi(const char *ssid, const char *password);
void saveWifiCredentials(const char *ssid, const char *password);
void loadWifiCredentials();
void setupAfterWifiConnected();

// Environment
void loadEnvironment();
void saveEnvironment();
void showEnvironmentScreen();

// Settings
void showSettingsScreen();
void loadSettings();
void saveSettings();

// Audio
void initAudio();
void playTone(int frequency, int durationMs);
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
    loadWifiCredentials();
    loadEnvironment();
    loadSettings();

    // Initialize audio
    initAudio();

    // Show boot screen
    showBootScreen();
    lv_timer_handler();  // Force LVGL to render the boot screen
    delay(5000);  // Display boot logo for 5 seconds

    // Fade out over 1 second (255 to 0 in 50 steps = 20ms per step)
    for (int i = 255; i >= 0; i -= 5) {
        display_set_brightness(i);
        delay(20);
    }

    // Check if we have stored WiFi credentials
    if (!storedWifiSSID.isEmpty()) {
        Serial.printf("Found stored WiFi credentials for: %s\n", storedWifiSSID.c_str());
        showConnectingScreen();
        lv_timer_handler();  // Render while still dark
        display_set_brightness(255);  // Now restore brightness

        // Try to connect with stored credentials
        connectToWifi(storedWifiSSID.c_str(), storedWifiPassword.c_str());

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Connected with stored WiFi credentials");
            // Continue to pairing/WebSocket setup
            setupAfterWifiConnected();
        } else {
            Serial.println("Stored WiFi credentials failed, showing setup screen");
            transitionTo(TerminalState::WIFI_SETUP);
        }
    } else {
        Serial.println("No stored WiFi credentials, showing setup screen");
        transitionTo(TerminalState::WIFI_SETUP);
        lv_timer_handler();  // Render while still dark
        display_set_brightness(255);  // Restore brightness
    }

    Serial.println("Setup complete!");
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
    // Handle LVGL (must be called frequently)
    lv_timer_handler();

    // Handle WebSocket (must be called frequently to maintain connection)
    if (currentState != TerminalState::BOOT &&
        currentState != TerminalState::WIFI_SETUP &&
        currentState != TerminalState::WIFI_PASSWORD &&
        currentState != TerminalState::PAIRING) {
        webSocket.loop();
    }

    // Send periodic heartbeat in all connected states
    if (currentState == TerminalState::IDLE ||
        currentState == TerminalState::SETTINGS ||
        currentState == TerminalState::QR_DISPLAY ||
        currentState == TerminalState::RESULT) {
        if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
            sendHeartbeat();
            lastHeartbeat = millis();
        }
    }

    // State-specific logic
    switch (currentState) {
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
    storeName = preferences.getString("store_name", "");

    Serial.printf("Config loaded - Server: %s, Terminal: %s, Store: %s, HasKey: %s\n",
                  ssimServerUrl.isEmpty() ? "(none)" : ssimServerUrl.c_str(),
                  terminalId.isEmpty() ? "(none)" : terminalId.c_str(),
                  storeName.isEmpty() ? "(none)" : storeName.c_str(),
                  apiKey.isEmpty() ? "no" : "yes");
}

void loadSettings() {
    displayBrightness = preferences.getInt("brightness", 255);
    soundEnabled = preferences.getBool("sound_on", true);
    Serial.printf("Settings loaded - Brightness: %d, Sound: %s\n",
                  displayBrightness, soundEnabled ? "on" : "off");
}

void saveSettings() {
    preferences.putInt("brightness", displayBrightness);
    preferences.putBool("sound_on", soundEnabled);
    Serial.printf("Settings saved - Brightness: %d, Sound: %s\n",
                  displayBrightness, soundEnabled ? "on" : "off");
}

void loadWifiCredentials() {
    storedWifiSSID = preferences.getString("wifi_ssid", "");
    storedWifiPassword = preferences.getString("wifi_pass", "");

    // Default WiFi credentials for testing (if none stored)
    if (storedWifiSSID.isEmpty()) {
        storedWifiSSID = "755Avenue_IOT";
        storedWifiPassword = "sun0sr0x";
        Serial.println("Using default WiFi credentials");
    }

    Serial.printf("WiFi credentials loaded - SSID: %s, HasPassword: %s\n",
                  storedWifiSSID.isEmpty() ? "(none)" : storedWifiSSID.c_str(),
                  storedWifiPassword.isEmpty() ? "no" : "yes");
}

void saveWifiCredentials(const char *ssid, const char *password) {
    preferences.putString("wifi_ssid", ssid);
    preferences.putString("wifi_pass", password);
    storedWifiSSID = ssid;
    storedWifiPassword = password;
    Serial.printf("WiFi credentials saved - SSID: %s\n", ssid);
}

void loadEnvironment() {
    int env = preferences.getInt("environment", 1);  // Default to production (1)
    currentEnvironment = (env == 1) ? Environment::PRODUCTION : Environment::DEVELOPMENT;
    Serial.printf("Environment loaded: %s\n", getServerConfig()->displayName);
}

void saveEnvironment() {
    preferences.putInt("environment", (currentEnvironment == Environment::PRODUCTION) ? 1 : 0);
    Serial.printf("Environment saved: %s\n", getServerConfig()->displayName);
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

        case TerminalState::WIFI_SETUP:
            showWifiSetupScreen();
            break;

        case TerminalState::WIFI_PASSWORD:
            showWifiPasswordScreen();
            break;

        case TerminalState::PAIRING:
            showPairingScreen();
            break;

        case TerminalState::CONNECTING:
            showConnectingScreen();
            break;

        case TerminalState::IDLE:
            showIdleScreen();
            currentPaymentId = "";
            currentQrData = "";
            currentAmount = 0;
            break;

        case TerminalState::SETTINGS:
            showSettingsScreen();
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
// WiFi Provisioning
// =============================================================================

/**
 * @brief Scan for available WiFi networks
 */
void scanWifiNetworks() {
    Serial.println("Scanning for WiFi networks...");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    Serial.printf("Found %d networks\n", n);

    wifiNetworkCount = 0;
    for (int i = 0; i < n && wifiNetworkCount < 20; i++) {
        // Skip duplicates and hidden networks
        String ssid = WiFi.SSID(i);
        if (ssid.isEmpty()) continue;

        bool duplicate = false;
        for (int j = 0; j < wifiNetworkCount; j++) {
            if (wifiNetworks[j] == ssid) {
                duplicate = true;
                break;
            }
        }

        if (!duplicate) {
            wifiNetworks[wifiNetworkCount] = ssid;
            Serial.printf("  [%d] %s (RSSI: %d)\n", wifiNetworkCount, ssid.c_str(), WiFi.RSSI(i));
            wifiNetworkCount++;
        }
    }

    WiFi.scanDelete();
}

/**
 * @brief Connect to a WiFi network with given credentials
 */
void connectToWifi(const char *ssid, const char *password) {
    Serial.printf("Connecting to WiFi: %s\n", ssid);

    WiFi.mode(WIFI_STA);
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
    }
}

/**
 * @brief Continue setup after WiFi is connected
 *        Handles pairing and WebSocket connection
 */
void setupAfterWifiConnected() {
    // Check if we have stored credentials
    if (apiKey.isEmpty()) {
        // No API key stored - need to pair
        Serial.println("No API key found, need to pair...");
        transitionTo(TerminalState::PAIRING);
    } else {
        Serial.printf("Using stored API key for terminal: %s\n", terminalId.c_str());
        connectWebSocket();
        transitionTo(TerminalState::IDLE);
    }
}

// =============================================================================
// WebSocket
// =============================================================================
void connectWebSocket() {
    const ServerConfig* config = getServerConfig();

    String wsPath = "/terminal/ws";
    if (!apiKey.isEmpty()) {
        wsPath += "?apiKey=" + apiKey;
    }

    Serial.printf("Connecting to WSS (%s): %s:%d%s\n",
                  config->displayName, config->wsHost, config->wsPort, wsPath.c_str());

    // Use SSL for wss://
    webSocket.beginSSL(config->wsHost, config->wsPort, wsPath.c_str());
    webSocket.onEvent(handleWebSocketEvent);
    webSocket.setReconnectInterval(RECONNECT_DELAY);

    // Enable automatic ping/pong to keep connection alive
    webSocket.enableHeartbeat(15000, 3000, 2);  // ping every 15s, timeout 3s, 2 retries

    // Skip SSL certificate validation (required for some servers)
    webSocket.setExtraHeaders("User-Agent: ssimTerminal/0.5.0");

    Serial.println("WebSocket connecting...");
}

// =============================================================================
// Pairing
// =============================================================================
bool pairWithServer(const char *pairingCode) {
    const ServerConfig* config = getServerConfig();
    Serial.printf("Pairing with code: %s (env: %s)\n", pairingCode, config->displayName);

    WiFiClientSecure client;
    client.setInsecure();  // Skip certificate validation for dev

    HTTPClient http;
    String pairUrl = String(config->apiHost) + "/api/terminal/pair";
    http.begin(client, pairUrl);
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
    Serial.printf("Pairing request to %s: %s\n", pairUrl.c_str(), requestBody.c_str());

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

            // Capture store name if provided
            if (!respDoc["storeName"].isNull()) {
                storeName = respDoc["storeName"].as<String>();
                preferences.putString("store_name", storeName);
            }

            // Save to NVS
            preferences.putString("terminal_id", terminalId);
            preferences.putString("api_key", apiKey);
            preferences.putString("server_url", config->wsHost);

            Serial.printf("Paired successfully! Terminal: %s, Store: %s\n",
                          terminalId.c_str(), storeName.c_str());
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
        Serial.println("Config update received");

        // Update store name if provided
        if (!p["storeName"].isNull()) {
            String newStoreName = p["storeName"].as<String>();
            if (newStoreName != storeName) {
                storeName = newStoreName;
                preferences.putString("store_name", storeName);
                Serial.printf("Store name updated: %s\n", storeName.c_str());
                // Refresh idle screen if we're on it
                if (currentState == TerminalState::IDLE) {
                    showIdleScreen();
                }
            }
        }
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

    // Logo container with subtle glow effect
    lv_obj_t *logo_circle = lv_obj_create(current_screen);
    lv_obj_set_size(logo_circle, 120, 120);
    lv_obj_align(logo_circle, LV_ALIGN_CENTER, 0, -60);
    lv_obj_set_style_bg_color(logo_circle, lv_color_hex(0x1976D2), 0);  // Blue
    lv_obj_set_style_radius(logo_circle, 60, 0);  // Full circle
    lv_obj_set_style_border_width(logo_circle, 0, 0);
    lv_obj_set_style_shadow_width(logo_circle, 40, 0);
    lv_obj_set_style_shadow_color(logo_circle, lv_color_hex(0x1976D2), 0);
    lv_obj_set_style_shadow_opa(logo_circle, LV_OPA_40, 0);

    // Dollar sign inside circle
    lv_obj_t *dollar = lv_label_create(logo_circle);
    lv_label_set_text(dollar, "$");
    lv_obj_set_style_text_color(dollar, lv_color_white(), 0);
    lv_obj_set_style_text_font(dollar, &lv_font_montserrat_48, 0);
    lv_obj_center(dollar);

    // App name
    lv_obj_t *name = lv_label_create(current_screen);
    lv_label_set_text(name, "ssimTerminal");
    lv_obj_set_style_text_color(name, lv_color_white(), 0);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_24, 0);
    lv_obj_align(name, LV_ALIGN_CENTER, 0, 30);

    // Tagline
    lv_obj_t *tagline = lv_label_create(current_screen);
    lv_label_set_text(tagline, "Payment Terminal");
    lv_obj_set_style_text_color(tagline, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(tagline, &lv_font_montserrat_14, 0);
    lv_obj_align(tagline, LV_ALIGN_CENTER, 0, 60);

    // Version at bottom
    lv_obj_t *version = lv_label_create(current_screen);
    lv_label_set_text_fmt(version, "v%s", FIRMWARE_VERSION);
    lv_obj_set_style_text_color(version, lv_color_hex(0x444444), 0);
    lv_obj_align(version, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// =============================================================================
// WiFi Setup UI
// =============================================================================

// LVGL keyboard for password entry
static lv_obj_t *wifi_keyboard = nullptr;
static lv_obj_t *wifi_password_ta = nullptr;

// LVGL keyboard for pairing code entry
static lv_obj_t *pairing_keyboard = nullptr;
static lv_obj_t *pairing_code_ta = nullptr;

/**
 * @brief Callback when a WiFi network button is clicked
 */
static void wifi_network_btn_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    int index = (int)(intptr_t)lv_obj_get_user_data(btn);

    if (index >= 0 && index < wifiNetworkCount) {
        selectedSSID = wifiNetworks[index];
        selectedNetworkIndex = index;
        Serial.printf("Selected network: %s\n", selectedSSID.c_str());

        // Transition to password entry state
        transitionTo(TerminalState::WIFI_PASSWORD);
    }
}

/**
 * @brief Callback for rescan button
 */
static void wifi_rescan_btn_cb(lv_event_t *e) {
    Serial.println("Rescanning WiFi networks...");
    showWifiSetupScreen();  // Re-show the screen (triggers rescan)
}

/**
 * @brief Callback for environment toggle button
 */
static void env_toggle_btn_cb(lv_event_t *e) {
    // Toggle environment
    if (currentEnvironment == Environment::DEVELOPMENT) {
        currentEnvironment = Environment::PRODUCTION;
    } else {
        currentEnvironment = Environment::DEVELOPMENT;
    }
    saveEnvironment();

    // Clear stored API key since we're changing environments
    preferences.remove("api_key");
    preferences.remove("terminal_id");
    apiKey = "";
    terminalId = "";
    Serial.printf("Environment changed to %s, credentials cleared\n", getServerConfig()->displayName);

    // Refresh the screen to show new environment
    showWifiSetupScreen();
}

/**
 * @brief Callback for back button on password screen
 */
static void wifi_back_btn_cb(lv_event_t *e) {
    Serial.println("Back button pressed - returning to network list");
    transitionTo(TerminalState::WIFI_SETUP);
}

/**
 * @brief Callback for keyboard events
 */
static void wifi_keyboard_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);

    // Only handle specific events we care about
    if (code == LV_EVENT_READY) {
        // User pressed Enter/OK
        const char *password = lv_textarea_get_text(wifi_password_ta);
        enteredPassword = password;
        Serial.printf("Keyboard OK pressed - connecting to %s\n", selectedSSID.c_str());

        // Show connecting screen
        showConnectingScreen();
        lv_timer_handler();

        // Try to connect
        connectToWifi(selectedSSID.c_str(), enteredPassword.c_str());

        if (WiFi.status() == WL_CONNECTED) {
            // Save credentials and continue
            saveWifiCredentials(selectedSSID.c_str(), enteredPassword.c_str());
            setupAfterWifiConnected();
        } else {
            // Connection failed - go back to WiFi setup
            showErrorScreen("Connection failed\nTap to retry");
            // After a delay, go back to WiFi setup
            delay(2000);
            lv_timer_handler();
            transitionTo(TerminalState::WIFI_SETUP);
        }
    } else if (code == LV_EVENT_CANCEL) {
        // User pressed keyboard cancel button - go back to network list
        Serial.println("Keyboard cancel pressed - returning to network list");
        transitionTo(TerminalState::WIFI_SETUP);
    }
    // Ignore all other events (LV_EVENT_PRESSED, LV_EVENT_VALUE_CHANGED, etc.)
}

/**
 * @brief Show WiFi network selection screen
 */
void showWifiSetupScreen() {
    Serial.println("UI: WiFi setup screen");

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_hex(0x1A1A1A), 0);

    // Environment toggle button (top-right corner)
    const ServerConfig* config = getServerConfig();
    lv_obj_t *env_btn = lv_btn_create(current_screen);
    lv_obj_set_size(env_btn, 90, 28);
    lv_obj_align(env_btn, LV_ALIGN_TOP_RIGHT, -10, 8);

    // Color based on environment
    if (currentEnvironment == Environment::PRODUCTION) {
        lv_obj_set_style_bg_color(env_btn, lv_color_hex(0xE65100), 0);  // Orange for production
    } else {
        lv_obj_set_style_bg_color(env_btn, lv_color_hex(0x1976D2), 0);  // Blue for dev
    }
    lv_obj_set_style_radius(env_btn, 14, 0);
    lv_obj_add_event_cb(env_btn, env_toggle_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *env_label = lv_label_create(env_btn);
    lv_label_set_text(env_label, config->displayName);
    lv_obj_set_style_text_color(env_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(env_label, &lv_font_montserrat_12, 0);
    lv_obj_center(env_label);

    // Title
    lv_obj_t *title = lv_label_create(current_screen);
    lv_label_set_text(title, "Select WiFi Network");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 14, 10);

    // Scanning indicator
    lv_obj_t *scanning = lv_label_create(current_screen);
    lv_label_set_text(scanning, "Scanning...");
    lv_obj_set_style_text_color(scanning, lv_color_hex(0x888888), 0);
    lv_obj_align(scanning, LV_ALIGN_TOP_MID, 0, 35);
    lv_timer_handler();

    // Scan for networks
    scanWifiNetworks();

    // Hide scanning label
    lv_obj_del(scanning);

    // Create scrollable list container
    lv_obj_t *list = lv_obj_create(current_screen);
    lv_obj_set_size(list, 340, 320);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 45);
    lv_obj_set_style_bg_color(list, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_radius(list, 8, 0);
    lv_obj_set_style_pad_all(list, 8, 0);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);

    if (wifiNetworkCount == 0) {
        lv_obj_t *no_networks = lv_label_create(list);
        lv_label_set_text(no_networks, "No networks found");
        lv_obj_set_style_text_color(no_networks, lv_color_hex(0x888888), 0);
    } else {
        // Create button for each network
        for (int i = 0; i < wifiNetworkCount; i++) {
            lv_obj_t *btn = lv_btn_create(list);
            lv_obj_set_size(btn, 320, 45);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x3A3A3A), 0);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x4A4A4A), LV_STATE_PRESSED);
            lv_obj_set_style_radius(btn, 6, 0);
            lv_obj_set_user_data(btn, (void*)(intptr_t)i);
            lv_obj_add_event_cb(btn, wifi_network_btn_cb, LV_EVENT_CLICKED, NULL);

            // WiFi icon
            lv_obj_t *icon = lv_label_create(btn);
            lv_label_set_text(icon, LV_SYMBOL_WIFI);
            lv_obj_set_style_text_color(icon, lv_color_hex(0x4CAF50), 0);
            lv_obj_align(icon, LV_ALIGN_LEFT_MID, 10, 0);

            // SSID label
            lv_obj_t *ssid_label = lv_label_create(btn);
            lv_label_set_text(ssid_label, wifiNetworks[i].c_str());
            lv_obj_set_style_text_color(ssid_label, lv_color_white(), 0);
            lv_label_set_long_mode(ssid_label, LV_LABEL_LONG_DOT);
            lv_obj_set_width(ssid_label, 250);
            lv_obj_align(ssid_label, LV_ALIGN_LEFT_MID, 45, 0);
        }
    }

    // Rescan button at bottom
    lv_obj_t *rescan_btn = lv_btn_create(current_screen);
    lv_obj_set_size(rescan_btn, 140, 40);
    lv_obj_align(rescan_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(rescan_btn, lv_color_hex(0x2196F3), 0);
    lv_obj_add_event_cb(rescan_btn, wifi_rescan_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *rescan_label = lv_label_create(rescan_btn);
    lv_label_set_text(rescan_label, LV_SYMBOL_REFRESH " Rescan");
    lv_obj_set_style_text_color(rescan_label, lv_color_white(), 0);
    lv_obj_center(rescan_label);
}

/**
 * @brief Show WiFi password entry screen
 */
void showWifiPasswordScreen() {
    Serial.printf("UI: WiFi password screen for %s\n", selectedSSID.c_str());

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_hex(0x1A1A1A), 0);

    // Back button (top-left)
    lv_obj_t *back_btn = lv_btn_create(current_screen);
    lv_obj_set_size(back_btn, 70, 32);
    lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 8);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x424242), 0);
    lv_obj_set_style_radius(back_btn, 6, 0);
    lv_obj_add_event_cb(back_btn, wifi_back_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(back_label, lv_color_white(), 0);
    lv_obj_center(back_label);

    // Title showing selected network
    lv_obj_t *title = lv_label_create(current_screen);
    lv_label_set_text(title, "Enter Password");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 20, 10);

    // Network name
    lv_obj_t *network_name = lv_label_create(current_screen);
    lv_label_set_text(network_name, selectedSSID.c_str());
    lv_obj_set_style_text_color(network_name, lv_color_hex(0x4CAF50), 0);
    lv_obj_align(network_name, LV_ALIGN_TOP_MID, 0, 38);

    // Password text area
    wifi_password_ta = lv_textarea_create(current_screen);
    lv_obj_set_size(wifi_password_ta, 340, 45);
    lv_obj_align(wifi_password_ta, LV_ALIGN_TOP_MID, 0, 70);
    lv_textarea_set_placeholder_text(wifi_password_ta, "Password");
    lv_textarea_set_password_mode(wifi_password_ta, true);
    lv_textarea_set_one_line(wifi_password_ta, true);
    lv_obj_set_style_bg_color(wifi_password_ta, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_text_color(wifi_password_ta, lv_color_white(), 0);
    lv_obj_set_style_border_color(wifi_password_ta, lv_color_hex(0x4CAF50), LV_STATE_FOCUSED);

    // Create keyboard
    wifi_keyboard = lv_keyboard_create(current_screen);
    lv_obj_set_size(wifi_keyboard, 368, 280);
    lv_obj_align(wifi_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(wifi_keyboard, wifi_password_ta);
    lv_obj_add_event_cb(wifi_keyboard, wifi_keyboard_cb, LV_EVENT_ALL, NULL);

    // Style the keyboard
    lv_obj_set_style_bg_color(wifi_keyboard, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_bg_color(wifi_keyboard, lv_color_hex(0x3A3A3A), LV_PART_ITEMS);
    lv_obj_set_style_text_color(wifi_keyboard, lv_color_white(), LV_PART_ITEMS);
}

/**
 * @brief Callback for pairing keyboard events
 */
static void pairing_keyboard_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_READY) {
        // User pressed Enter/OK
        const char *pairingCode = lv_textarea_get_text(pairing_code_ta);
        Serial.printf("Pairing code entered: %s\n", pairingCode);

        if (strlen(pairingCode) < 4) {
            // Code too short
            Serial.println("Pairing code too short");
            return;
        }

        // Show connecting screen
        showConnectingScreen();
        lv_timer_handler();

        // Try to pair with server
        if (pairWithServer(pairingCode)) {
            // Success - connect WebSocket and go to idle
            connectWebSocket();
            transitionTo(TerminalState::IDLE);
        } else {
            // Failed - show error and return to pairing screen
            showErrorScreen("Pairing failed\nCheck code and try again");
            delay(2000);
            lv_timer_handler();
            transitionTo(TerminalState::PAIRING);
        }
    } else if (code == LV_EVENT_CANCEL) {
        // User pressed cancel - go back to WiFi setup
        Serial.println("Pairing cancelled - returning to WiFi setup");
        transitionTo(TerminalState::WIFI_SETUP);
    }
}

void showPairingScreen() {
    Serial.println("UI: Pairing screen");

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_hex(0x1A1A1A), 0);

    // Back button (top-left) - return to WiFi setup
    lv_obj_t *back_btn = lv_btn_create(current_screen);
    lv_obj_set_size(back_btn, 70, 32);
    lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 8);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x424242), 0);
    lv_obj_set_style_radius(back_btn, 6, 0);
    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
        Serial.println("Back to WiFi setup");
        transitionTo(TerminalState::WIFI_SETUP);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(back_label, lv_color_white(), 0);
    lv_obj_center(back_label);

    // Environment indicator (top-right)
    const ServerConfig* config = getServerConfig();
    lv_obj_t *env_label = lv_label_create(current_screen);
    lv_label_set_text(env_label, config->displayName);
    lv_obj_set_style_text_color(env_label,
        currentEnvironment == Environment::PRODUCTION ? lv_color_hex(0xE65100) : lv_color_hex(0x1976D2), 0);
    lv_obj_set_style_text_font(env_label, &lv_font_montserrat_12, 0);
    lv_obj_align(env_label, LV_ALIGN_TOP_RIGHT, -14, 14);

    // Title
    lv_obj_t *title = lv_label_create(current_screen);
    lv_label_set_text(title, "Enter Pairing Code");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Instructions
    lv_obj_t *instructions = lv_label_create(current_screen);
    lv_label_set_text(instructions, "Get pairing code from SSIM\nSettings > Terminals > Add Terminal");
    lv_obj_set_style_text_color(instructions, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(instructions, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(instructions, LV_ALIGN_TOP_MID, 0, 38);

    // Pairing code text area
    pairing_code_ta = lv_textarea_create(current_screen);
    lv_obj_set_size(pairing_code_ta, 280, 55);
    lv_obj_align(pairing_code_ta, LV_ALIGN_TOP_MID, 0, 90);
    lv_textarea_set_placeholder_text(pairing_code_ta, "XXXXXX");
    lv_textarea_set_one_line(pairing_code_ta, true);
    lv_textarea_set_max_length(pairing_code_ta, 12);
    lv_obj_set_style_bg_color(pairing_code_ta, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_text_color(pairing_code_ta, lv_color_white(), 0);
    lv_obj_set_style_text_font(pairing_code_ta, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_align(pairing_code_ta, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_border_color(pairing_code_ta, lv_color_hex(0x4CAF50), LV_STATE_FOCUSED);

    // Create keyboard (number mode for pairing codes)
    pairing_keyboard = lv_keyboard_create(current_screen);
    lv_obj_set_size(pairing_keyboard, 368, 260);
    lv_obj_align(pairing_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(pairing_keyboard, pairing_code_ta);
    lv_keyboard_set_mode(pairing_keyboard, LV_KEYBOARD_MODE_TEXT_UPPER);  // Uppercase for codes
    lv_obj_add_event_cb(pairing_keyboard, pairing_keyboard_cb, LV_EVENT_ALL, NULL);

    // Style the keyboard
    lv_obj_set_style_bg_color(pairing_keyboard, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_bg_color(pairing_keyboard, lv_color_hex(0x3A3A3A), LV_PART_ITEMS);
    lv_obj_set_style_text_color(pairing_keyboard, lv_color_white(), LV_PART_ITEMS);
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

static void idle_settings_btn_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.println("Settings button pressed");
        transitionTo(TerminalState::SETTINGS);
    }
}

void showIdleScreen() {
    Serial.println("UI: Idle screen");

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_black(), 0);

    // Settings gear button (top-right)
    lv_obj_t *settings_btn = lv_btn_create(current_screen);
    lv_obj_set_size(settings_btn, 44, 44);
    lv_obj_align(settings_btn, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(settings_btn, lv_color_hex(0x333333), 0);
    lv_obj_set_style_bg_opa(settings_btn, LV_OPA_80, 0);
    lv_obj_set_style_radius(settings_btn, 22, 0);
    lv_obj_set_style_shadow_width(settings_btn, 0, 0);
    lv_obj_add_event_cb(settings_btn, idle_settings_btn_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *gear_icon = lv_label_create(settings_btn);
    lv_label_set_text(gear_icon, LV_SYMBOL_SETTINGS);
    lv_obj_set_style_text_color(gear_icon, lv_color_white(), 0);
    lv_obj_set_style_text_font(gear_icon, &lv_font_montserrat_20, 0);
    lv_obj_center(gear_icon);

    // Main title
    lv_obj_t *title = lv_label_create(current_screen);
    lv_label_set_text(title, "Ready for Payment");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_center(title);

    lv_obj_t *sub = lv_label_create(current_screen);
    lv_label_set_text(sub, "Waiting for transaction...");
    lv_obj_set_style_text_color(sub, lv_color_hex(0x888888), 0);
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, 40);

    // Store name at bottom (like firmware version on boot screen)
    // Fall back to server hostname if no store name from server
    lv_obj_t *store_label = lv_label_create(current_screen);
    if (!storeName.isEmpty()) {
        lv_label_set_text(store_label, storeName.c_str());
    } else {
        const ServerConfig* config = getServerConfig();
        lv_label_set_text(store_label, config->wsHost);
    }
    lv_obj_set_style_text_color(store_label, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(store_label, &lv_font_montserrat_14, 0);
    lv_obj_align(store_label, LV_ALIGN_BOTTOM_MID, 0, -15);
}

// =============================================================================
// Settings Screen
// =============================================================================
static lv_obj_t *brightness_slider = nullptr;
static lv_obj_t *sound_switch = nullptr;

static void settings_back_btn_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Settings: Back button pressed");
        transitionTo(TerminalState::IDLE);
    }
}

static void settings_brightness_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *slider = lv_event_get_target(e);
        displayBrightness = lv_slider_get_value(slider);
        display_set_brightness(displayBrightness);
        Serial.printf("Settings: Brightness = %d\n", displayBrightness);
    }
}

static void settings_sound_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *sw = lv_event_get_target(e);
        soundEnabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
        Serial.printf("Settings: Sound = %s\n", soundEnabled ? "on" : "off");
    }
}

static void settings_wifi_btn_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Settings: WiFi button pressed");
        saveSettings();  // Save settings before leaving
        transitionTo(TerminalState::WIFI_SETUP);
    }
}

static void settings_save_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        saveSettings();
        Serial.println("Settings saved");
        transitionTo(TerminalState::IDLE);
    }
}

void showSettingsScreen() {
    Serial.println("UI: Settings screen");

    switchScreen();

    lv_obj_set_style_bg_color(current_screen, lv_color_hex(0x1A1A1A), 0);

    // Header with back button
    lv_obj_t *back_btn = lv_btn_create(current_screen);
    lv_obj_set_size(back_btn, 70, 36);
    lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x424242), 0);
    lv_obj_set_style_radius(back_btn, 6, 0);
    lv_obj_add_event_cb(back_btn, settings_back_btn_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(back_label, lv_color_white(), 0);
    lv_obj_center(back_label);

    // Title
    lv_obj_t *title = lv_label_create(current_screen);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    int y_offset = 70;

    // Brightness section
    lv_obj_t *bright_label = lv_label_create(current_screen);
    lv_label_set_text(bright_label, "Brightness");
    lv_obj_set_style_text_color(bright_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(bright_label, LV_ALIGN_TOP_LEFT, 20, y_offset);

    brightness_slider = lv_slider_create(current_screen);
    lv_obj_set_size(brightness_slider, LCD_WIDTH - 60, 20);
    lv_obj_align(brightness_slider, LV_ALIGN_TOP_MID, 0, y_offset + 25);
    lv_slider_set_range(brightness_slider, 20, 255);  // Min 20 to prevent completely dark
    lv_slider_set_value(brightness_slider, displayBrightness, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0x444444), LV_PART_MAIN);
    lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0x4A90D9), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(brightness_slider, lv_color_white(), LV_PART_KNOB);
    lv_obj_add_event_cb(brightness_slider, settings_brightness_handler, LV_EVENT_VALUE_CHANGED, NULL);

    y_offset += 70;

    // Sound section
    lv_obj_t *sound_label = lv_label_create(current_screen);
    lv_label_set_text(sound_label, "Sound Effects");
    lv_obj_set_style_text_color(sound_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(sound_label, LV_ALIGN_TOP_LEFT, 20, y_offset);

    sound_switch = lv_switch_create(current_screen);
    lv_obj_set_size(sound_switch, 60, 30);
    lv_obj_align(sound_switch, LV_ALIGN_TOP_RIGHT, -20, y_offset - 5);
    if (soundEnabled) {
        lv_obj_add_state(sound_switch, LV_STATE_CHECKED);
    }
    lv_obj_set_style_bg_color(sound_switch, lv_color_hex(0x444444), LV_PART_MAIN);
    lv_obj_set_style_bg_color(sound_switch, lv_color_hex(0x4CAF50), (int)LV_PART_INDICATOR | (int)LV_STATE_CHECKED);
    lv_obj_add_event_cb(sound_switch, settings_sound_handler, LV_EVENT_VALUE_CHANGED, NULL);

    y_offset += 60;

    // WiFi Settings button
    lv_obj_t *wifi_btn = lv_btn_create(current_screen);
    lv_obj_set_size(wifi_btn, LCD_WIDTH - 40, 44);
    lv_obj_align(wifi_btn, LV_ALIGN_TOP_MID, 0, y_offset);
    lv_obj_set_style_bg_color(wifi_btn, lv_color_hex(0x333333), 0);
    lv_obj_set_style_radius(wifi_btn, 8, 0);
    lv_obj_add_event_cb(wifi_btn, settings_wifi_btn_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *wifi_icon = lv_label_create(wifi_btn);
    lv_label_set_text(wifi_icon, LV_SYMBOL_WIFI "  WiFi Settings");
    lv_obj_set_style_text_color(wifi_icon, lv_color_white(), 0);
    lv_obj_center(wifi_icon);

    y_offset += 60;

    // Environment indicator
    const ServerConfig* config = getServerConfig();
    lv_obj_t *env_label = lv_label_create(current_screen);
    char env_text[64];
    snprintf(env_text, sizeof(env_text), "Environment: %s", config->displayName);
    lv_label_set_text(env_label, env_text);
    lv_obj_set_style_text_color(env_label, lv_color_hex(0x888888), 0);
    lv_obj_align(env_label, LV_ALIGN_TOP_LEFT, 20, y_offset);

    y_offset += 30;

    // Terminal info
    lv_obj_t *info_label = lv_label_create(current_screen);
    char info_text[128];
    snprintf(info_text, sizeof(info_text), "Firmware: v%s\nTerminal: %.8s...",
             FIRMWARE_VERSION, terminalId.isEmpty() ? "Not paired" : terminalId.c_str());
    lv_label_set_text(info_label, info_text);
    lv_obj_set_style_text_color(info_label, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(info_label, &lv_font_montserrat_12, 0);
    lv_obj_align(info_label, LV_ALIGN_TOP_LEFT, 20, y_offset);

    // Save button at bottom
    lv_obj_t *save_btn = lv_btn_create(current_screen);
    lv_obj_set_size(save_btn, LCD_WIDTH - 40, 48);
    lv_obj_align(save_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(save_btn, lv_color_hex(0x4A90D9), 0);
    lv_obj_set_style_radius(save_btn, 8, 0);
    lv_obj_add_event_cb(save_btn, settings_save_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *save_label = lv_label_create(save_btn);
    lv_label_set_text(save_label, "Save & Return");
    lv_obj_set_style_text_color(save_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(save_label, &lv_font_montserrat_16, 0);
    lv_obj_center(save_label);
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
    lv_color_t icon_bg_color;
    const char *icon_text;
    const char *status_text;

    if (strcmp(status, "approved") == 0) {
        bg_color = lv_color_hex(0x0D3B0D);       // Very dark green background
        icon_bg_color = lv_color_hex(0x2E7D32); // Green circle
        icon_text = LV_SYMBOL_OK;
        status_text = "Payment Approved";
    } else if (strcmp(status, "declined") == 0) {
        bg_color = lv_color_hex(0x3B0D0D);       // Very dark red background
        icon_bg_color = lv_color_hex(0xC62828); // Red circle
        icon_text = LV_SYMBOL_CLOSE;
        status_text = "Payment Declined";
    } else if (strcmp(status, "cancelled") == 0) {
        bg_color = lv_color_hex(0x1A1A1A);       // Dark gray background
        icon_bg_color = lv_color_hex(0x616161); // Gray circle
        icon_text = LV_SYMBOL_CLOSE;
        status_text = "Payment Cancelled";
    } else if (strcmp(status, "expired") == 0) {
        bg_color = lv_color_hex(0x2B1A00);       // Very dark orange background
        icon_bg_color = lv_color_hex(0xE65100); // Orange circle
        icon_text = LV_SYMBOL_REFRESH;
        status_text = "Payment Expired";
    } else {
        bg_color = lv_color_hex(0x2B1A00);       // Very dark orange background
        icon_bg_color = lv_color_hex(0xE65100); // Orange circle
        icon_text = LV_SYMBOL_WARNING;
        status_text = "Payment Error";
    }

    lv_obj_set_style_bg_color(current_screen, bg_color, 0);

    // Circular background for icon
    lv_obj_t *icon_circle = lv_obj_create(current_screen);
    lv_obj_set_size(icon_circle, 140, 140);
    lv_obj_align(icon_circle, LV_ALIGN_CENTER, 0, -60);
    lv_obj_set_style_bg_color(icon_circle, icon_bg_color, 0);
    lv_obj_set_style_radius(icon_circle, 70, 0);  // Full circle
    lv_obj_set_style_border_width(icon_circle, 0, 0);
    lv_obj_set_style_shadow_width(icon_circle, 30, 0);
    lv_obj_set_style_shadow_color(icon_circle, icon_bg_color, 0);
    lv_obj_set_style_shadow_opa(icon_circle, LV_OPA_50, 0);

    // Icon inside circle
    lv_obj_t *icon = lv_label_create(icon_circle);
    lv_label_set_text(icon, icon_text);
    lv_obj_set_style_text_color(icon, lv_color_white(), 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_48, 0);
    lv_obj_center(icon);

    // Status text
    lv_obj_t *status_label = lv_label_create(current_screen);
    lv_label_set_text(status_label, status_text);
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_24, 0);
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 55);

    // Message (if provided)
    if (strlen(message) > 0) {
        lv_obj_t *msg_label = lv_label_create(current_screen);
        lv_label_set_text(msg_label, message);
        lv_obj_set_style_text_color(msg_label, lv_color_hex(0xAAAAAA), 0);
        lv_obj_set_style_text_align(msg_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(msg_label, LV_ALIGN_CENTER, 0, 95);
    }

    // Play audio feedback
    playResultSound(status);
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
// Audio - ES8311 Codec via I2S
// =============================================================================
static bool audioInitialized = false;
static const int SAMPLE_RATE = 44100;
static const int I2S_PORT = I2S_NUM_0;

// ES8311 I2C registers
#define ES8311_RESET_REG        0x00
#define ES8311_CLK_MANAGER_REG1 0x01
#define ES8311_CLK_MANAGER_REG2 0x02
#define ES8311_CLK_MANAGER_REG3 0x03
#define ES8311_DAC_REG32        0x32
#define ES8311_SYSTEM_REG0D     0x0D
#define ES8311_SYSTEM_REG0E     0x0E
#define ES8311_SYSTEM_REG12     0x12
#define ES8311_SYSTEM_REG13     0x13

static bool es8311WriteReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(AUDIO_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

void initAudio() {
    Serial.println("Audio: Initializing ES8311 codec...");

    // Reset ES8311
    es8311WriteReg(ES8311_RESET_REG, 0x3F);
    delay(20);
    es8311WriteReg(ES8311_RESET_REG, 0x00);
    delay(20);

    // Configure clock manager for I2S slave mode
    es8311WriteReg(ES8311_CLK_MANAGER_REG1, 0x30);  // MCLK from I2S BCLK
    es8311WriteReg(ES8311_CLK_MANAGER_REG2, 0x10);  // Clock divider

    // Enable DAC and analog output
    es8311WriteReg(ES8311_SYSTEM_REG0D, 0x01);  // Power up analog
    es8311WriteReg(ES8311_SYSTEM_REG0E, 0x02);  // Enable DAC
    es8311WriteReg(ES8311_SYSTEM_REG12, 0x00);  // Unmute DAC
    es8311WriteReg(ES8311_SYSTEM_REG13, 0x10);  // Enable output

    // Configure DAC volume
    es8311WriteReg(ES8311_DAC_REG32, 0xBF);  // Set volume (0xBF = reasonable level)

    // Configure I2S
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRCLK,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_DIN
    };

    esp_err_t err = i2s_driver_install((i2s_port_t)I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Audio: I2S driver install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin((i2s_port_t)I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Audio: I2S pin config failed: %d\n", err);
        return;
    }

    audioInitialized = true;
    Serial.println("Audio: ES8311 codec initialized");
}

void playTone(int frequency, int durationMs) {
    if (!audioInitialized || !soundEnabled) return;

    const int numSamples = (SAMPLE_RATE * durationMs) / 1000;
    const float amplitude = 8000.0f;  // Volume (max 32767 for 16-bit)
    const float twoPiF = 2.0f * PI * frequency / SAMPLE_RATE;

    int16_t *samples = (int16_t *)heap_caps_malloc(512 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!samples) {
        Serial.println("Audio: Failed to allocate sample buffer");
        return;
    }

    int samplesWritten = 0;
    while (samplesWritten < numSamples) {
        int batchSize = min(256, numSamples - samplesWritten);

        for (int i = 0; i < batchSize; i++) {
            float sample = amplitude * sinf(twoPiF * (samplesWritten + i));
            // Stereo: duplicate sample for left and right channels
            samples[i * 2] = (int16_t)sample;      // Left
            samples[i * 2 + 1] = (int16_t)sample;  // Right
        }

        size_t bytesWritten = 0;
        i2s_write((i2s_port_t)I2S_PORT, samples, batchSize * 4, &bytesWritten, portMAX_DELAY);
        samplesWritten += batchSize;
    }

    heap_caps_free(samples);
}

void playResultSound(const char *status) {
    if (!soundEnabled) {
        Serial.printf("Audio: Sound disabled, skipping %s sound\n", status);
        return;
    }

    Serial.printf("Audio: Playing %s sound\n", status);

    if (strcmp(status, "approved") == 0) {
        // Happy ascending tones
        playTone(523, 100);  // C5
        delay(50);
        playTone(659, 100);  // E5
        delay(50);
        playTone(784, 150);  // G5
    } else if (strcmp(status, "declined") == 0) {
        // Sad descending tones
        playTone(392, 200);  // G4
        delay(100);
        playTone(294, 300);  // D4
    } else if (strcmp(status, "cancelled") == 0) {
        // Single neutral beep
        playTone(440, 200);  // A4
    } else if (strcmp(status, "expired") == 0) {
        // Two short beeps
        playTone(349, 100);  // F4
        delay(100);
        playTone(349, 100);  // F4
    } else {
        // Default: single beep
        playTone(440, 150);  // A4
    }
}
