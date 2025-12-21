/**
 * @file display.cpp
 * @brief Display driver implementation for ssimTerminal
 *
 * Supports:
 * - ESP32-S3-Touch-AMOLED-1.8: SH8601 QSPI AMOLED with TCA9554 expander
 * - ESP32-S3-Touch-LCD-7: ST7262 RGB LCD with CH422G expander
 */

#include "display.h"
#include "pins_config.h"
#include <Wire.h>
#include <Arduino_GFX_Library.h>

// =============================================================================
// Display Configuration
// =============================================================================

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
// Board-Specific I/O Expander Code
// =============================================================================

#if DISPLAY_TYPE_QSPI
// TCA9554 I2C GPIO Expander (for AMOLED boards)

static void tca9554_write(uint8_t value) {
    Wire.beginTransmission(EXPANDER_I2C_ADDR);
    Wire.write(0x01);  // Output port register
    Wire.write(value);
    Wire.endTransmission();
}

static void tca9554_config(uint8_t config) {
    Wire.beginTransmission(EXPANDER_I2C_ADDR);
    Wire.write(0x03);  // Configuration register
    Wire.write(config);
    Wire.endTransmission();
}

static bool init_expander() {
    Wire.beginTransmission(EXPANDER_I2C_ADDR);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.printf("ERROR: TCA9554 not responding at 0x%02X (error %d)\n",
                      EXPANDER_I2C_ADDR, error);
        return false;
    }

    // Configure pins 0-3 as outputs
    tca9554_config(0xF0);

    // Reset sequence: assert reset (low), wait, release (high)
    tca9554_write(0x00);
    delay(20);
    tca9554_write(0x07);  // P0, P1, P2 = HIGH
    delay(50);

    Serial.println("TCA9554 expander initialized, display/touch reset complete");
    return true;
}

#elif DISPLAY_TYPE_RGB
// CH422G I2C GPIO Expander (for RGB LCD boards)
// CH422G uses multiple I2C addresses for different operations:
//   0x24 (WR_SET) - Configuration register
//   0x38 (WR_IO)  - Write to IO pins 0-7
//   0x23 (WR_OC)  - Write to IO pins 8-11
//   0x26 (RD_IO)  - Read IO pins

#define CH422G_ADDR_WR_SET  0x24  // Configuration register
#define CH422G_ADDR_WR_IO   0x38  // IO pins 0-7 output
#define CH422G_ADDR_WR_OC   0x23  // IO pins 8-11 output (open-collector)

// CH422G pin assignments for ESP32-S3-Touch-LCD-7:
// IO0 = not used
// IO1 = TP_RST (touch reset)
// IO2 = LCD_BL (backlight)
// IO3 = LCD_RST (LCD reset)
// IO4 = SD_CS (directly wired, not used via expander)
// IO5 = USB_SEL
// IO6 = PA_EN (if present)

#define CH422G_PIN_TP_RST   (1 << 1)  // IO1
#define CH422G_PIN_LCD_BL   (1 << 2)  // IO2
#define CH422G_PIN_LCD_RST  (1 << 3)  // IO3

static bool init_expander() {
    Serial.println("Initializing CH422G I/O expander...");

    // Check if CH422G responds at config address
    Wire.beginTransmission(CH422G_ADDR_WR_SET);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.printf("ERROR: CH422G not responding at 0x%02X (error %d)\n",
                      CH422G_ADDR_WR_SET, error);
        return false;
    }

    // Step 1: Enable output mode for all IO pins
    // WR_SET register: bit0=OD_EN, bit1=IO_OE (set to 1 to enable IO output)
    Wire.beginTransmission(CH422G_ADDR_WR_SET);
    Wire.write(0x01);  // Enable push-pull output mode (OD_EN=1, IO_OE=0)
    Wire.endTransmission();
    delay(10);

    // Step 2: Set IO pins - LCD_RST low first for reset
    Wire.beginTransmission(CH422G_ADDR_WR_IO);
    Wire.write(CH422G_PIN_TP_RST);  // TP_RST=HIGH, LCD_BL=LOW, LCD_RST=LOW
    Wire.endTransmission();
    delay(20);

    // Step 3: Release LCD reset and enable backlight
    Wire.beginTransmission(CH422G_ADDR_WR_IO);
    Wire.write(CH422G_PIN_TP_RST | CH422G_PIN_LCD_BL | CH422G_PIN_LCD_RST);
    Wire.endTransmission();
    delay(50);

    Serial.println("CH422G initialized: TP_RST=HIGH, LCD_BL=HIGH, LCD_RST=HIGH");
    return true;
}

#endif

// =============================================================================
// I2C Bus Scan
// =============================================================================

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

// =============================================================================
// LVGL Display Flush Callback
// =============================================================================

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);

    lv_disp_flush_ready(drv);
}

#if DISPLAY_TYPE_QSPI
// SH8601 requires coordinates to be even
static void lvgl_rounder_cb(lv_disp_drv_t *drv, lv_area_t *area) {
    area->x1 = area->x1 & ~1;
    area->y1 = area->y1 & ~1;
    area->x2 = area->x2 | 1;
    area->y2 = area->y2 | 1;
}
#endif

// =============================================================================
// Display Initialization
// =============================================================================

#if DISPLAY_TYPE_QSPI
// SH8601 QSPI AMOLED initialization

static bool init_display_hardware() {
    Serial.println("Initializing SH8601 QSPI AMOLED...");

    // Create QSPI bus
    bus = new Arduino_ESP32QSPI(
        LCD_QSPI_CS,
        LCD_QSPI_CLK,
        LCD_QSPI_D0,
        LCD_QSPI_D1,
        LCD_QSPI_D2,
        LCD_QSPI_D3
    );

    // Create display driver (using Waveshare's modified Arduino_GFX)
    gfx = new Arduino_SH8601(bus, -1 /* rst */, 0 /* rotation */, false /* ips */,
                              LCD_WIDTH, LCD_HEIGHT);

    if (!gfx->begin()) {
        Serial.println("ERROR: SH8601 initialization failed");
        return false;
    }

    // Fill with white first (like Waveshare demo)
    gfx->fillScreen(WHITE);

    // Fade in brightness
    Serial.println("Fading in brightness...");
    for (int i = 0; i <= 255; i += 5) {
        gfx->Display_Brightness(i);
        delay(10);
    }
    Serial.println("Display brightness at max");

    // Clear to black before LVGL takes over
    gfx->fillScreen(0x0000);

    return true;
}

#elif DISPLAY_TYPE_RGB
// ST7262 RGB LCD initialization
// Note: Arduino_ESP32RGBPanel requires ESP_ARDUINO_VERSION_MAJOR < 3
// The pioarduino platform uses Arduino 3.x, so we need esp_lcd directly

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"

static esp_lcd_panel_handle_t panel_handle = nullptr;
static uint16_t *rgb_framebuffer = nullptr;

// Custom GFX wrapper for RGB display
class Arduino_RGB_GFX : public Arduino_GFX {
public:
    Arduino_RGB_GFX(int16_t w, int16_t h, uint16_t *fb)
        : Arduino_GFX(w, h), _fb(fb), _w(w), _h(h) {}

    bool begin(int32_t speed = 0) override { return true; }

    void writePixelPreclipped(int16_t x, int16_t y, uint16_t color) override {
        if (_fb) {
            _fb[y * _w + x] = color;
        }
    }

    void writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
        if (!_fb) return;
        for (int16_t j = 0; j < h; j++) {
            for (int16_t i = 0; i < w; i++) {
                _fb[(y + j) * _w + (x + i)] = color;
            }
        }
    }

    void draw16bitRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h) override {
        if (!_fb) return;
        for (int16_t j = 0; j < h; j++) {
            memcpy(&_fb[(y + j) * _w + x], &bitmap[j * w], w * 2);
        }
    }

    void displayOn() override {}
    void displayOff() override {}

private:
    uint16_t *_fb;
    int16_t _w, _h;
};

static bool init_display_hardware() {
    Serial.println("Initializing ST7262 RGB LCD via esp_lcd...");

    // Configure RGB panel
    esp_lcd_rgb_panel_config_t panel_config = {};
    panel_config.clk_src = LCD_CLK_SRC_DEFAULT;
    panel_config.timings.pclk_hz = LCD_PCLK_FREQ;
    panel_config.timings.h_res = LCD_WIDTH;
    panel_config.timings.v_res = LCD_HEIGHT;
    panel_config.timings.hsync_back_porch = LCD_HSYNC_BACK_PORCH;
    panel_config.timings.hsync_front_porch = LCD_HSYNC_FRONT_PORCH;
    panel_config.timings.hsync_pulse_width = LCD_HSYNC_PULSE_WIDTH;
    panel_config.timings.vsync_back_porch = LCD_VSYNC_BACK_PORCH;
    panel_config.timings.vsync_front_porch = LCD_VSYNC_FRONT_PORCH;
    panel_config.timings.vsync_pulse_width = LCD_VSYNC_PULSE_WIDTH;
    panel_config.timings.flags.pclk_active_neg = 1;
    panel_config.data_width = 16;
    panel_config.num_fbs = 1;
    panel_config.bounce_buffer_size_px = LCD_WIDTH * 10;  // Bounce buffer for stability
    panel_config.hsync_gpio_num = LCD_HSYNC;
    panel_config.vsync_gpio_num = LCD_VSYNC;
    panel_config.de_gpio_num = LCD_DE;
    panel_config.pclk_gpio_num = LCD_PCLK;
    panel_config.disp_gpio_num = -1;
    // RGB565 data pins: B[3:7], G[2:7], R[3:7] = 5+6+5 = 16 bits
    panel_config.data_gpio_nums[0] = LCD_B3;
    panel_config.data_gpio_nums[1] = LCD_B4;
    panel_config.data_gpio_nums[2] = LCD_B5;
    panel_config.data_gpio_nums[3] = LCD_B6;
    panel_config.data_gpio_nums[4] = LCD_B7;
    panel_config.data_gpio_nums[5] = LCD_G2;
    panel_config.data_gpio_nums[6] = LCD_G3;
    panel_config.data_gpio_nums[7] = LCD_G4;
    panel_config.data_gpio_nums[8] = LCD_G5;
    panel_config.data_gpio_nums[9] = LCD_G6;
    panel_config.data_gpio_nums[10] = LCD_G7;
    panel_config.data_gpio_nums[11] = LCD_R3;
    panel_config.data_gpio_nums[12] = LCD_R4;
    panel_config.data_gpio_nums[13] = LCD_R5;
    panel_config.data_gpio_nums[14] = LCD_R6;
    panel_config.data_gpio_nums[15] = LCD_R7;
    panel_config.flags.fb_in_psram = 1;

    esp_err_t ret = esp_lcd_new_rgb_panel(&panel_config, &panel_handle);
    if (ret != ESP_OK) {
        Serial.printf("ERROR: esp_lcd_new_rgb_panel failed: %d\n", ret);
        return false;
    }

    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        Serial.printf("ERROR: esp_lcd_panel_reset failed: %d\n", ret);
        return false;
    }

    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        Serial.printf("ERROR: esp_lcd_panel_init failed: %d\n", ret);
        return false;
    }

    // Get the framebuffer
    void *fb = nullptr;
    esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 1, &fb);
    rgb_framebuffer = (uint16_t *)fb;

    if (!rgb_framebuffer) {
        Serial.println("ERROR: Failed to get RGB framebuffer");
        return false;
    }

    // Create GFX wrapper
    gfx = new Arduino_RGB_GFX(LCD_WIDTH, LCD_HEIGHT, rgb_framebuffer);

    // Clear to black
    memset(rgb_framebuffer, 0, LCD_WIDTH * LCD_HEIGHT * 2);

    Serial.println("ST7262 RGB LCD initialized successfully");
    return true;
}

#endif

// =============================================================================
// Public API
// =============================================================================

bool display_init() {
    Serial.println("Initializing display...");
    Serial.printf("Board: %s\n", BOARD_NAME);
    Serial.printf("Resolution: %dx%d\n", LCD_WIDTH, LCD_HEIGHT);

    // Initialize I2C for expander
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    // Scan I2C bus
    i2c_scan();

    // Initialize expander and reset display
    if (!init_expander()) {
        Serial.println("WARNING: Failed to initialize I/O expander");
        // Continue anyway - display might still work
    }

    // Initialize display hardware
    if (!init_display_hardware()) {
        Serial.println("ERROR: Failed to initialize display hardware");
        return false;
    }

    Serial.printf("Display initialized: %dx%d\n", LCD_WIDTH, LCD_HEIGHT);

    // Allocate LVGL draw buffers in PSRAM
    buf1 = (lv_color_t *)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    buf2 = (lv_color_t *)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);

    if (!buf1 || !buf2) {
        Serial.println("ERROR: Failed to allocate LVGL display buffers in PSRAM");
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
#if DISPLAY_TYPE_QSPI
    disp_drv.rounder_cb = lvgl_rounder_cb;  // SH8601 requires even coordinates
#endif
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
#if DISPLAY_TYPE_QSPI
    if (gfx) {
        gfx->Display_Brightness(brightness);
    }
#else
    // RGB LCD: backlight is on/off via expander, no PWM brightness
    // Could be implemented via PWM if connected to GPIO
    (void)brightness;
#endif
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
