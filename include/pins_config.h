/**
 * @file pins_config.h
 * @brief Pin definitions for Waveshare ESP32-S3-Touch-AMOLED-1.8
 *
 * Hardware: https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8
 *
 * Pin assignments sourced from:
 * - LVGL Forum: https://forum.lvgl.io/t/sh8601-with-lvgl-9-waveshare-esp32-s3-touch-amoled-1-8/21818
 * - Waveshare demo code
 *
 * NOTE: The display reset is controlled via TCA9554 I2C GPIO expander, not directly.
 */

#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

#include <Arduino.h>

// =============================================================================
// Display - SH8601 AMOLED (QSPI Interface)
// =============================================================================
#define LCD_WIDTH           368
#define LCD_HEIGHT          448

// QSPI pins for SH8601 (confirmed from Waveshare demo)
#define LCD_QSPI_CLK        11        // QSPI Clock
#define LCD_QSPI_D0         4         // QSPI Data 0
#define LCD_QSPI_D1         5         // QSPI Data 1
#define LCD_QSPI_D2         6         // QSPI Data 2
#define LCD_QSPI_D3         7         // QSPI Data 3
#define LCD_QSPI_CS         12        // Chip Select

// Reset is via TCA9554 I2C expander (not direct GPIO)
#define LCD_RST_VIA_EXPANDER  1       // Flag indicating reset via expander
#define LCD_RST             -1        // Not directly connected

// Display settings
#define LCD_QSPI_FREQ       40000000  // 40MHz QSPI clock

// =============================================================================
// I2C Bus (Shared by touch, expander, PMU, sensors)
// =============================================================================
#define I2C_SDA             1         // I2C Data
#define I2C_SCL             2         // I2C Clock
#define I2C_FREQ            400000    // 400kHz I2C speed

// =============================================================================
// Touch Controller - FT3168 (I2C Interface)
// =============================================================================
#define TOUCH_I2C_ADDR      0x38      // FT3168 I2C address (7-bit)
#define TOUCH_INT           3         // Touch interrupt pin
#define TOUCH_RST           -1        // Reset via TCA9554 expander

// =============================================================================
// I/O Expander - TCA9554 (I2C Interface)
// =============================================================================
#define EXPANDER_I2C_ADDR   0x20      // TCA9554 I2C address

// TCA9554 output pin assignments (accent these through expander)
#define EXP_PIN_LCD_RST     1         // LCD reset (active low)
#define EXP_PIN_TP_RST      2         // Touch panel reset
#define EXP_PIN_SD_CS       3         // SD card chip select

// =============================================================================
// Audio - ES8311 Codec (I2C + I2S Interface)
// =============================================================================
#define AUDIO_I2C_ADDR      0x18      // ES8311 I2C address

// I2S pins (for sound feedback)
#define I2S_BCLK            45        // Bit clock
#define I2S_LRCLK           46        // Left/Right clock (Word Select)
#define I2S_DOUT            40        // Data out to codec (speaker)
#define I2S_DIN             48        // Data in from codec (microphone)

// Speaker amplifier enable (may be via expander)
#define SPK_EN              -1        // Check schematic

// =============================================================================
// Power Management - AXP2101 (I2C Interface)
// =============================================================================
#define PMU_I2C_ADDR        0x34      // AXP2101 I2C address
#define PMU_IRQ             -1        // PMU interrupt (check schematic)

// =============================================================================
// IMU - QMI8658 6-axis (I2C Interface)
// =============================================================================
#define IMU_I2C_ADDR        0x6B      // QMI8658 I2C address
#define IMU_INT1            -1        // Interrupt 1 (check schematic)
#define IMU_INT2            -1        // Interrupt 2 (check schematic)

// =============================================================================
// RTC - PCF85063 (I2C Interface)
// =============================================================================
#define RTC_I2C_ADDR        0x51      // PCF85063 I2C address

// =============================================================================
// SD Card (SPI Interface via TCA9554 CS)
// =============================================================================
#define SD_MOSI             13        // SPI MOSI
#define SD_MISO             47        // SPI MISO
#define SD_SCLK             14        // SPI Clock
// SD_CS is via TCA9554 expander pin EXP_PIN_SD_CS

// =============================================================================
// Buttons
// =============================================================================
#define BTN_BOOT            0         // Boot button (active low)
#define BTN_PWR             -1        // Power button (check schematic)

// =============================================================================
// Status LED - WS2812 (if present)
// =============================================================================
#define LED_PIN             -1        // RGB LED data pin (check schematic)

// =============================================================================
// Battery ADC
// =============================================================================
#define BAT_ADC             -1        // Battery voltage via AXP2101

#endif // PINS_CONFIG_H
