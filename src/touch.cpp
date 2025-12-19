/**
 * @file touch.cpp
 * @brief Touch driver implementation for FT3168
 *
 * FT3168 is a FocalTech capacitive touch controller compatible with FT5x06 protocol.
 * Uses I2C communication at address 0x38.
 */

#include "touch.h"
#include "pins_config.h"
#include <Wire.h>

// =============================================================================
// FT3168 Register Definitions
// =============================================================================

#define FT_REG_DEV_MODE     0x00    // Device mode
#define FT_REG_GEST_ID      0x01    // Gesture ID
#define FT_REG_TD_STATUS    0x02    // Touch points status
#define FT_REG_P1_XH        0x03    // Point 1 X high byte
#define FT_REG_P1_XL        0x04    // Point 1 X low byte
#define FT_REG_P1_YH        0x05    // Point 1 Y high byte
#define FT_REG_P1_YL        0x06    // Point 1 Y low byte
#define FT_REG_P1_WEIGHT    0x07    // Point 1 weight
#define FT_REG_P1_MISC      0x08    // Point 1 misc
#define FT_REG_THGROUP      0x80    // Threshold group
#define FT_REG_CHIP_ID      0xA3    // Chip vendor ID
#define FT_REG_FIRM_VER     0xA6    // Firmware version
#define FT_REG_VENDOR_ID    0xA8    // Focal Vendor ID

#define FT_CHIP_ID_VALUE    0x64    // Expected chip ID for FT3168

// =============================================================================
// Static Variables
// =============================================================================

static lv_indev_drv_t indev_drv;
static lv_indev_t *indev = nullptr;
static bool touch_initialized = false;
static uint16_t last_x = 0;
static uint16_t last_y = 0;

// =============================================================================
// I2C Communication
// =============================================================================

/**
 * @brief Read bytes from FT3168 register
 */
static bool ft_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    Wire.requestFrom(TOUCH_I2C_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }

    return true;
}

/**
 * @brief Write byte to FT3168 register
 */
static bool ft_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

// =============================================================================
// LVGL Touchpad Read Callback
// =============================================================================

/**
 * @brief LVGL touchpad read callback
 */
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
    // Just verify the touch controller is responding

    uint8_t chip_id = 0;
    if (!ft_read_reg(FT_REG_CHIP_ID, &chip_id, 1)) {
        Serial.println("ERROR: Failed to communicate with touch controller");
        return false;
    }

    Serial.printf("Touch controller chip ID: 0x%02X\n", chip_id);

    // Read firmware version
    uint8_t fw_ver = 0;
    ft_read_reg(FT_REG_FIRM_VER, &fw_ver, 1);
    Serial.printf("Touch controller firmware: 0x%02X\n", fw_ver);

    // Set touch threshold (optional)
    // ft_write_reg(FT_REG_THGROUP, 22);

    // Configure touch interrupt (if using)
    if (TOUCH_INT >= 0) {
        pinMode(TOUCH_INT, INPUT);
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

    // Check for interrupt if pin is configured (faster)
    if (TOUCH_INT >= 0) {
        if (digitalRead(TOUCH_INT) == HIGH) {
            // No touch event
            *pressed = false;
            return true;
        }
    }

    // Read touch status
    uint8_t buf[7];
    if (!ft_read_reg(FT_REG_TD_STATUS, buf, 7)) {
        return false;
    }

    uint8_t touch_count = buf[0] & 0x0F;

    if (touch_count == 0 || touch_count > 5) {
        *pressed = false;
        return true;
    }

    // Parse first touch point
    // X: buf[1] bits 0-3 = high, buf[2] = low
    // Y: buf[3] bits 0-3 = high, buf[4] = low
    uint16_t raw_x = ((buf[1] & 0x0F) << 8) | buf[2];
    uint16_t raw_y = ((buf[3] & 0x0F) << 8) | buf[4];

    // Clamp to display bounds
    if (raw_x >= LCD_WIDTH) raw_x = LCD_WIDTH - 1;
    if (raw_y >= LCD_HEIGHT) raw_y = LCD_HEIGHT - 1;

    *x = raw_x;
    *y = raw_y;
    *pressed = true;

    return true;
}

lv_indev_t* touch_get_driver() {
    return indev;
}
