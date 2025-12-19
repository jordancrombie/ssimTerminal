/**
 * @file display.cpp
 * @brief Display driver implementation for Waveshare ESP32-S3-Touch-AMOLED-1.8
 *
 * Uses Arduino_GFX library for SH8601 QSPI AMOLED display.
 * The display reset is controlled via TCA9554 I2C GPIO expander.
 */

#include "display.h"
#include "pins_config.h"
#include <Wire.h>
#include <Arduino_GFX_Library.h>

// =============================================================================
// Display Configuration
// =============================================================================

// QSPI bus for SH8601
static Arduino_DataBus *bus = nullptr;
static Arduino_GFX *gfx = nullptr;

// LVGL display buffer (in PSRAM)
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;
static lv_disp_drv_t disp_drv;
static lv_disp_t *disp = nullptr;

// Buffer size: 1/10 of screen for good performance
#define DRAW_BUF_LINES  (LCD_HEIGHT / 10)
#define DRAW_BUF_SIZE   (LCD_WIDTH * DRAW_BUF_LINES)

// =============================================================================
// TCA9554 I2C GPIO Expander
// =============================================================================

/**
 * @brief Write to TCA9554 output register
 * @param value Output pin states (bit 0 = P0, bit 7 = P7)
 */
static void tca9554_write(uint8_t value) {
    Wire.beginTransmission(EXPANDER_I2C_ADDR);
    Wire.write(0x01);  // Output port register
    Wire.write(value);
    Wire.endTransmission();
}

/**
 * @brief Configure TCA9554 pin directions
 * @param config 0 = output, 1 = input (per bit)
 */
static void tca9554_config(uint8_t config) {
    Wire.beginTransmission(EXPANDER_I2C_ADDR);
    Wire.write(0x03);  // Configuration register
    Wire.write(config);
    Wire.endTransmission();
}

/**
 * @brief Scan I2C bus and print found devices
 */
static void i2c_scan() {
    Serial.println("Scanning I2C bus...");
    int count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X\n", addr);
            count++;
        }
    }
    Serial.printf("I2C scan complete: %d devices found\n", count);
}

/**
 * @brief Initialize TCA9554 and reset display/touch
 * Based on Waveshare demo code - uses P0, P1, P2 for resets
 */
static bool init_expander() {
    // Scan I2C bus first to see what's connected
    i2c_scan();

    // Check if expander responds
    Wire.beginTransmission(EXPANDER_I2C_ADDR);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.printf("ERROR: TCA9554 not responding at 0x%02X (error %d)\n",
                      EXPANDER_I2C_ADDR, error);
        Serial.println("Display reset will not work without expander!");
        return false;
    }

    // Configure pins 0-3 as outputs (Waveshare uses P0, P1, P2)
    tca9554_config(0xF0);  // P0-P3 outputs, P4-P7 inputs

    // Reset sequence: assert reset (low), wait, release (high)
    // Waveshare demo: P0=?, P1=LCD_RST, P2=TP_RST
    uint8_t outputs = 0x00;  // All low (P0, P1, P2 = LOW)
    tca9554_write(outputs);
    delay(20);

    outputs = 0x07;  // P0, P1, P2 = HIGH (all resets released)
    tca9554_write(outputs);
    delay(50);

    Serial.println("TCA9554 expander initialized, display/touch reset complete");
    return true;
}

// =============================================================================
// LVGL Display Flush Callback
// =============================================================================

/**
 * @brief Flush pixel data to display (called by LVGL)
 */
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Use draw16bitRGBBitmap for efficient block transfer
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);

    lv_disp_flush_ready(drv);
}

/**
 * @brief Rounder callback for SH8601 (coordinates must be even)
 *
 * The SH8601 requires x_start, y_start, x_end, y_end to be divisible by 2.
 */
static void lvgl_rounder_cb(lv_disp_drv_t *drv, lv_area_t *area) {
    area->x1 = area->x1 & ~1;  // Round down to even
    area->y1 = area->y1 & ~1;
    area->x2 = area->x2 | 1;   // Round up to odd (so width is even)
    area->y2 = area->y2 | 1;
}

// =============================================================================
// Public API
// =============================================================================

bool display_init() {
    Serial.println("Initializing display...");

    // Initialize I2C for expander
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    // Initialize expander and reset display
    if (!init_expander()) {
        Serial.println("ERROR: Failed to initialize TCA9554 expander");
        return false;
    }

    // Create QSPI bus for SH8601
    // Arduino_ESP32QSPI(cs, sck, d0, d1, d2, d3)
    bus = new Arduino_ESP32QSPI(
        LCD_QSPI_CS,
        LCD_QSPI_CLK,
        LCD_QSPI_D0,
        LCD_QSPI_D1,
        LCD_QSPI_D2,
        LCD_QSPI_D3
    );

    // Create display driver (using Waveshare's modified Arduino_GFX)
    // Arduino_SH8601(bus, rst, rotation, ips, width, height)
    // Note: rst=-1 because we handle reset via TCA9554
    gfx = new Arduino_SH8601(bus, -1 /* rst */, 0 /* rotation */, false /* ips */,
                              LCD_WIDTH, LCD_HEIGHT);

    // Initialize with default speed (let library choose)
    if (!gfx->begin()) {
        Serial.println("ERROR: Display initialization failed");
        return false;
    }
    Serial.println("Display driver initialized");

    // Fill with white first (like Waveshare demo)
    gfx->fillScreen(WHITE);

    // Fade in brightness (like Waveshare demo)
    Serial.println("Fading in brightness...");
    for (int i = 0; i <= 255; i += 5) {
        gfx->Display_Brightness(i);
        delay(10);
    }
    Serial.println("Display brightness at max");

    // Visual test - draw colored rectangles to verify display works
    Serial.println("Drawing test pattern...");
    gfx->fillScreen(0xF800);  // RED
    delay(500);
    gfx->fillScreen(0x07E0);  // GREEN
    delay(500);
    gfx->fillScreen(0x001F);  // BLUE
    delay(500);
    gfx->fillScreen(0x0000);  // BLACK
    Serial.println("Test pattern complete");

    Serial.printf("Display initialized: %dx%d\n", LCD_WIDTH, LCD_HEIGHT);

    // Allocate LVGL draw buffers in PSRAM
    buf1 = (lv_color_t *)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    buf2 = (lv_color_t *)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);

    if (!buf1 || !buf2) {
        Serial.println("ERROR: Failed to allocate LVGL display buffers in PSRAM");
        // Try regular RAM as fallback
        buf1 = (lv_color_t *)malloc(DRAW_BUF_SIZE * sizeof(lv_color_t));
        buf2 = (lv_color_t *)malloc(DRAW_BUF_SIZE * sizeof(lv_color_t));
        if (!buf1 || !buf2) {
            Serial.println("ERROR: Failed to allocate LVGL display buffers");
            return false;
        }
        Serial.println("Warning: Using heap RAM for display buffers (slower)");
    }

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, DRAW_BUF_SIZE);

    // Initialize LVGL display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.rounder_cb = lvgl_rounder_cb;  // SH8601 requires even coordinates
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = false;

    disp = lv_disp_drv_register(&disp_drv);

    if (!disp) {
        Serial.println("ERROR: Failed to register LVGL display driver");
        return false;
    }

    Serial.println("LVGL display driver registered");
    return true;
}

void display_set_brightness(uint8_t brightness) {
    // AMOLED brightness is typically controlled via commands
    // For SH8601, this may require specific command sequences
    // TODO: Implement brightness control via display commands
    Serial.printf("Display brightness set to: %d\n", brightness);
}

void display_power(bool on) {
    if (gfx) {
        if (on) {
            gfx->displayOn();
            Serial.println("Display ON");
        } else {
            gfx->displayOff();
            Serial.println("Display OFF");
        }
    }
}

lv_disp_t* display_get_driver() {
    return disp;
}
