/**
 * @file touch.cpp
 * @brief Touch driver implementation for ssimTerminal
 *
 * Supports:
 * - FT3168 (on ESP32-S3-Touch-AMOLED-1.8): FocalTech capacitive touch at 0x38
 * - GT911 (on ESP32-S3-Touch-LCD-7): Goodix capacitive touch at 0x5D
 * - CST816 (on ESP32-S3-Touch-LCD-1.85C-BOX): Hynitron capacitive touch at 0x15
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
// CST816 Touch Controller (LCD-1.85C-BOX)
// =============================================================================

#if TOUCH_DRIVER_CST816

// CST816 register definitions
#define CST816_REG_GESTURE      0x01    // Gesture ID
#define CST816_REG_FINGER_NUM   0x02    // Number of touch points
#define CST816_REG_XH           0x03    // X high byte (bits 0-3) + event flag (bits 6-7)
#define CST816_REG_XL           0x04    // X low byte
#define CST816_REG_YH           0x05    // Y high byte (bits 0-3)
#define CST816_REG_YL           0x06    // Y low byte
#define CST816_REG_CHIP_ID      0xA7    // Chip ID register
#define CST816_REG_FW_VER       0xA9    // Firmware version
#define CST816_REG_DIS_AUTOSLEEP 0xFE   // Disable auto-sleep

// TCA9554 register definitions
#define TCA9554_OUTPUT_REG      0x01

static bool cst_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
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

static bool cst_write_reg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission() == 0;
}

// Read TCA9554 output register
static uint8_t tca9554_read_output() {
    Wire.beginTransmission(EXPANDER_I2C_ADDR);
    Wire.write(TCA9554_OUTPUT_REG);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)EXPANDER_I2C_ADDR, (uint8_t)1);
    return Wire.read();
}

// Write TCA9554 output register
static void tca9554_write_output(uint8_t value) {
    Wire.beginTransmission(EXPANDER_I2C_ADDR);
    Wire.write(TCA9554_OUTPUT_REG);
    Wire.write(value);
    Wire.endTransmission();
}

// Set a single TCA9554 pin without affecting others
static void tca9554_set_pin(uint8_t pin_bit, bool high) {
    uint8_t current = tca9554_read_output();
    if (high) {
        current |= (1 << pin_bit);
    } else {
        current &= ~(1 << pin_bit);
    }
    tca9554_write_output(current);
}

// Hardware reset CST816 via TCA9554 expander
static void cst816_hw_reset() {
    Serial.println("Resetting CST816 via TCA9554...");
    tca9554_set_pin(EXP_PIN_TP_RST, false);  // Assert reset (low)
    delay(10);
    tca9554_set_pin(EXP_PIN_TP_RST, true);   // Release reset (high)
    delay(50);  // Wait for touch controller to initialize
}

// Disable auto-sleep mode
static void cst816_disable_autosleep() {
    // Reset first to ensure proper state
    cst816_hw_reset();
    // Write a non-zero value to disable auto-sleep
    uint8_t value = 10;  // Waveshare uses this value
    if (cst_write_reg(CST816_REG_DIS_AUTOSLEEP, value)) {
        Serial.println("CST816 auto-sleep disabled");
    } else {
        Serial.println("WARNING: Failed to disable CST816 auto-sleep");
    }
}

static bool touch_hw_init() {
    Serial.println("Initializing CST816 touch controller...");

    // Hardware reset via TCA9554 expander
    cst816_hw_reset();

    // Configure interrupt pin as input with pullup
    #if TOUCH_INT >= 0
    pinMode(TOUCH_INT, INPUT_PULLUP);
    Serial.printf("Touch interrupt on GPIO %d\n", TOUCH_INT);
    #endif

    // Check if device responds
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    uint8_t error = Wire.endTransmission();

    if (error != 0) {
        // Try alternate address
        #ifdef TOUCH_I2C_ADDR_ALT
        Serial.printf("CST816 not at 0x%02X, trying alternate 0x%02X...\n",
                      TOUCH_I2C_ADDR, TOUCH_I2C_ADDR_ALT);
        Wire.beginTransmission(TOUCH_I2C_ADDR_ALT);
        error = Wire.endTransmission();
        #endif

        if (error != 0) {
            Serial.printf("ERROR: CST816 not found (error %d)\n", error);
            return false;
        }
    }

    Serial.printf("CST816 found at 0x%02X\n", TOUCH_I2C_ADDR);

    // Disable auto-sleep (must be done after reset)
    cst816_disable_autosleep();

    // Read chip ID
    uint8_t chip_id = 0;
    if (!cst_read_reg(CST816_REG_CHIP_ID, &chip_id, 1)) {
        Serial.println("WARNING: Failed to read CST816 chip ID");
    } else {
        Serial.printf("CST816 chip ID: 0x%02X\n", chip_id);
    }

    // Read firmware version
    uint8_t fw_ver = 0;
    cst_read_reg(CST816_REG_FW_VER, &fw_ver, 1);
    Serial.printf("CST816 firmware: 0x%02X\n", fw_ver);

    Serial.println("CST816 initialized");
    return true;
}

static bool touch_hw_read(uint16_t *x, uint16_t *y, bool *pressed) {
    // Read touch data (6 bytes starting from FINGER_NUM)
    uint8_t buf[6];
    if (!cst_read_reg(CST816_REG_FINGER_NUM, buf, 6)) {
        *pressed = false;
        return false;
    }

    // buf[0] = finger count
    // buf[1] = X high (bits 0-3) + event flag (bits 6-7)
    // buf[2] = X low
    // buf[3] = Y high (bits 0-3)
    // buf[4] = Y low

    uint8_t finger_count = buf[0] & 0x0F;

    if (finger_count == 0) {
        *pressed = false;
        return true;
    }

    // Check event flag (bits 6-7 of XH register)
    // 0 = press down, 1 = lift up, 2 = contact
    uint8_t event_flag = (buf[1] >> 6) & 0x03;
    if (event_flag == 1) {  // Lift up - no touch
        *pressed = false;
        return true;
    }

    // Extract coordinates
    uint16_t raw_x = ((buf[1] & 0x0F) << 8) | buf[2];
    uint16_t raw_y = ((buf[3] & 0x0F) << 8) | buf[4];

    // Filter edge touches (round display may have phantom touches at edges)
    const uint16_t EDGE_MARGIN = 5;
    if (raw_x < EDGE_MARGIN || raw_x >= (LCD_WIDTH - EDGE_MARGIN) ||
        raw_y < EDGE_MARGIN || raw_y >= (LCD_HEIGHT - EDGE_MARGIN)) {
        *pressed = false;
        return true;
    }

    // Clamp to display bounds
    if (raw_x >= LCD_WIDTH) raw_x = LCD_WIDTH - 1;
    if (raw_y >= LCD_HEIGHT) raw_y = LCD_HEIGHT - 1;

    *x = raw_x;
    *y = raw_y;
    *pressed = true;

    return true;
}

#endif // TOUCH_DRIVER_CST816

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
