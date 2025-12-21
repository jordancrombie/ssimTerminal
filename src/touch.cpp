/**
 * @file touch.cpp
 * @brief Touch driver implementation for ssimTerminal
 *
 * Supports:
 * - FT3168 (on ESP32-S3-Touch-AMOLED-1.8): FocalTech capacitive touch at 0x38
 * - GT911 (on ESP32-S3-Touch-LCD-7): Goodix capacitive touch at 0x5D
 */

#include "touch.h"
#include "pins_config.h"
#include <Wire.h>

// =============================================================================
// Static Variables
// =============================================================================

static lv_indev_drv_t indev_drv;
static lv_indev_t *indev = nullptr;
static bool touch_initialized = false;
static uint16_t last_x = 0;
static uint16_t last_y = 0;

// =============================================================================
// FT3168 Touch Controller (AMOLED boards)
// =============================================================================

#if TOUCH_DRIVER_FT3168

#define FT_REG_DEV_MODE     0x00
#define FT_REG_GEST_ID      0x01
#define FT_REG_TD_STATUS    0x02
#define FT_REG_P1_XH        0x03
#define FT_REG_CHIP_ID      0xA3
#define FT_REG_FIRM_VER     0xA6

static bool ft_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }

    return true;
}

static bool touch_hw_init() {
    Serial.println("Initializing FT3168 touch controller...");

    uint8_t chip_id = 0;
    if (!ft_read_reg(FT_REG_CHIP_ID, &chip_id, 1)) {
        Serial.println("ERROR: Failed to communicate with FT3168");
        return false;
    }

    Serial.printf("FT3168 chip ID: 0x%02X\n", chip_id);

    uint8_t fw_ver = 0;
    ft_read_reg(FT_REG_FIRM_VER, &fw_ver, 1);
    Serial.printf("FT3168 firmware: 0x%02X\n", fw_ver);

    return true;
}

static bool touch_hw_read(uint16_t *x, uint16_t *y, bool *pressed) {
    uint8_t buf[7];
    if (!ft_read_reg(FT_REG_TD_STATUS, buf, 7)) {
        *pressed = false;
        return false;
    }

    uint8_t touch_count = buf[0] & 0x0F;

    if (touch_count == 0 || touch_count > 5) {
        *pressed = false;
        return true;
    }

    // Check event flag (bits 6-7 of buf[1])
    uint8_t event_flag = (buf[1] >> 6) & 0x03;
    if (event_flag == 1 || event_flag == 3) {
        *pressed = false;
        return true;
    }

    uint16_t raw_x = ((buf[1] & 0x0F) << 8) | buf[2];
    uint16_t raw_y = ((buf[3] & 0x0F) << 8) | buf[4];

    // Filter edge touches
    const uint16_t EDGE_MARGIN = 5;
    if (raw_x < EDGE_MARGIN || raw_x >= (LCD_WIDTH - EDGE_MARGIN) ||
        raw_y < EDGE_MARGIN || raw_y >= (LCD_HEIGHT - EDGE_MARGIN)) {
        *pressed = false;
        return true;
    }

    if (raw_x >= LCD_WIDTH) raw_x = LCD_WIDTH - 1;
    if (raw_y >= LCD_HEIGHT) raw_y = LCD_HEIGHT - 1;

    *x = raw_x;
    *y = raw_y;
    *pressed = true;

    return true;
}

#endif // TOUCH_DRIVER_FT3168

// =============================================================================
// GT911 Touch Controller (RGB LCD boards)
// =============================================================================

#if TOUCH_DRIVER_GT911

#define GT911_POINT_INFO    0x814E
#define GT911_POINT1        0x814F
#define GT911_CONFIG        0x8047
#define GT911_PRODUCT_ID    0x8140
#define GT911_FIRMWARE_VER  0x8144

static bool gt_read_reg(uint16_t reg, uint8_t *data, uint8_t len) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write((reg >> 8) & 0xFF);  // High byte
    Wire.write(reg & 0xFF);         // Low byte
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }

    return true;
}

static bool gt_write_reg(uint16_t reg, uint8_t value) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write((reg >> 8) & 0xFF);
    Wire.write(reg & 0xFF);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool touch_hw_init() {
    Serial.println("Initializing GT911 touch controller...");

    // Reset touch controller if RST pin is available
    #if TOUCH_RST >= 0
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(50);
    #endif

    // Try primary address
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        // Try alternate address
        Serial.printf("GT911 not at 0x%02X, trying 0x%02X...\n",
                      TOUCH_I2C_ADDR, TOUCH_I2C_ADDR_ALT);
        // Note: Would need to update TOUCH_I2C_ADDR dynamically
        return false;
    }

    // Read product ID (4 bytes, should be "911" in ASCII)
    uint8_t product_id[4] = {0};
    if (!gt_read_reg(GT911_PRODUCT_ID, product_id, 4)) {
        Serial.println("ERROR: Failed to read GT911 product ID");
        return false;
    }

    Serial.printf("GT911 Product ID: %c%c%c%c\n",
                  product_id[0], product_id[1], product_id[2], product_id[3]);

    // Read firmware version
    uint8_t fw_ver[2] = {0};
    gt_read_reg(GT911_FIRMWARE_VER, fw_ver, 2);
    Serial.printf("GT911 Firmware: 0x%02X%02X\n", fw_ver[1], fw_ver[0]);

    return true;
}

static bool touch_hw_read(uint16_t *x, uint16_t *y, bool *pressed) {
    // Read point info register
    uint8_t point_info = 0;
    if (!gt_read_reg(GT911_POINT_INFO, &point_info, 1)) {
        *pressed = false;
        return false;
    }

    // Bit 7: buffer status (1 = data ready), Bits 0-3: number of touch points
    bool buffer_ready = (point_info & 0x80) != 0;
    uint8_t touch_count = point_info & 0x0F;

    if (!buffer_ready) {
        *pressed = false;
        return true;
    }

    // Clear buffer status by writing 0 to point info register
    gt_write_reg(GT911_POINT_INFO, 0);

    if (touch_count == 0 || touch_count > 5) {
        *pressed = false;
        return true;
    }

    // Read first touch point (8 bytes: track_id, x_l, x_h, y_l, y_h, size_l, size_h, reserved)
    uint8_t point_data[8] = {0};
    if (!gt_read_reg(GT911_POINT1, point_data, 8)) {
        *pressed = false;
        return false;
    }

    // Parse coordinates (little-endian)
    uint16_t raw_x = point_data[1] | (point_data[2] << 8);
    uint16_t raw_y = point_data[3] | (point_data[4] << 8);

    // Clamp to display bounds
    if (raw_x >= LCD_WIDTH) raw_x = LCD_WIDTH - 1;
    if (raw_y >= LCD_HEIGHT) raw_y = LCD_HEIGHT - 1;

    *x = raw_x;
    *y = raw_y;
    *pressed = true;

    return true;
}

#endif // TOUCH_DRIVER_GT911

// =============================================================================
// LVGL Touchpad Read Callback
// =============================================================================

static void lvgl_touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    uint16_t x, y;
    bool pressed;

    if (touch_read(&x, &y, &pressed)) {
        if (pressed) {
            data->state = LV_INDEV_STATE_PR;
            data->point.x = x;
            data->point.y = y;
            last_x = x;
            last_y = y;
        } else {
            data->state = LV_INDEV_STATE_REL;
            data->point.x = last_x;
            data->point.y = last_y;
        }
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// =============================================================================
// Public API
// =============================================================================

bool touch_init() {
    Serial.println("Initializing touch controller...");

    // I2C should already be initialized by display module
    if (!touch_hw_init()) {
        Serial.println("ERROR: Touch hardware init failed");
        return false;
    }

    // Initialize LVGL input device driver
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touchpad_read;
    indev = lv_indev_drv_register(&indev_drv);

    if (!indev) {
        Serial.println("ERROR: Failed to register LVGL input device");
        return false;
    }

    touch_initialized = true;
    Serial.println("Touch controller initialized");
    return true;
}

bool touch_read(uint16_t *x, uint16_t *y, bool *pressed) {
    if (!touch_initialized) {
        *pressed = false;
        return false;
    }

    return touch_hw_read(x, y, pressed);
}

lv_indev_t* touch_get_driver() {
    return indev;
}
