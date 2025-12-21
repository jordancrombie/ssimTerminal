/**
 * @file pins_config.h
 * @brief Pin configuration compatibility layer for ssimTerminal
 *
 * This file includes the board-specific configuration based on PlatformIO
 * build flags. All pin definitions are now in include/boards/*.h files.
 *
 * Supported boards:
 * - ESP32-S3-Touch-AMOLED-1.8: -DBOARD_WAVESHARE_AMOLED_1_8
 * - ESP32-S3-Touch-LCD-7: -DBOARD_WAVESHARE_LCD_7
 */

#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

#include <Arduino.h>
#include "boards/board_config.h"

// =============================================================================
// Additional common definitions (not board-specific)
// =============================================================================

// IMU - QMI8658 6-axis (I2C Interface) - only on some boards
#ifndef IMU_I2C_ADDR
#define IMU_I2C_ADDR        0x6B      // QMI8658 I2C address
#endif
#ifndef IMU_INT1
#define IMU_INT1            -1        // Interrupt 1
#endif
#ifndef IMU_INT2
#define IMU_INT2            -1        // Interrupt 2
#endif

// RTC - PCF85063 (I2C Interface) - only on some boards
#ifndef RTC_I2C_ADDR
#define RTC_I2C_ADDR        0x51      // PCF85063 I2C address
#endif

// Power button (if present)
#ifndef BTN_PWR
#define BTN_PWR             -1        // Power button
#endif

// Status LED - WS2812 (if present)
#ifndef LED_PIN
#define LED_PIN             -1        // RGB LED data pin
#endif

// Battery ADC (usually via PMU)
#ifndef BAT_ADC
#define BAT_ADC             -1        // Battery voltage ADC
#endif

#endif // PINS_CONFIG_H
