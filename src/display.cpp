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
 * @brief Initialize TCA9554 and reset display/touch
 */
static bool init_expander() {
    // Configure pins 0-3 as outputs (0x00 = all outputs for lower nibble)
    tca9554_config(0xF0);  // P0-P3 outputs, P4-P7 inputs

    // Reset sequence: assert reset (low), wait, release (high)
    // Bit 1 = LCD_RST, Bit 2 = TP_RST
    uint8_t outputs = 0x00;  // All low
    tca9554_write(outputs);
    delay(20);

    outputs = (1 << EXP_PIN_LCD_RST) | (1 << EXP_PIN_TP_RST);  // Release resets
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

    #ifdef DIRECT_MODE
    // Direct mode - LVGL manages the full framebuffer
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
    #else
    // Normal mode - partial updates
    gfx->startWrite();
    gfx->writeAddrWindow(area->x1, area->y1, w, h);
    gfx->writePixels((uint16_t *)color_p, w * h);
    gfx->endWrite();
    #endif

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

    // Create display driver
    // Arduino_SH8601(bus, rst, r, ips, w, h)
    // Note: rst=-1 because we handle reset via TCA9554
    gfx = new Arduino_SH8601(bus, -1 /* rst */, 0 /* rotation */, false /* ips */,
                              LCD_WIDTH, LCD_HEIGHT);

    if (!gfx->begin(LCD_QSPI_FREQ)) {
        Serial.println("ERROR: Display initialization failed");
        return false;
    }

    // Clear screen to black
    gfx->fillScreen(BLACK);
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
