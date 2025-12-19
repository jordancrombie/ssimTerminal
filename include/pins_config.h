/**
 * @file pins_config.h
 * @brief Pin definitions for Waveshare ESP32-S3-Touch-AMOLED-1.8
 *
 * Hardware: https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8
 *
 * NOTE: These pin assignments are based on Waveshare documentation and
 * community examples. Verify against the official demo code if issues arise:
 * https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8-Demo.zip
 */

#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// =============================================================================
// Display - SH8601 AMOLED (QSPI Interface)
// =============================================================================
#define LCD_WIDTH           368
#define LCD_HEIGHT          448

// QSPI pins for SH8601
#define LCD_SCLK            47    // QSPI Clock
#define LCD_SDIO0           18    // QSPI Data 0
#define LCD_SDIO1           7     // QSPI Data 1
#define LCD_SDIO2           48    // QSPI Data 2
#define LCD_SDIO3           5     // QSPI Data 3
#define LCD_CS              6     // Chip Select
#define LCD_RST             17    // Reset (active low)
#define LCD_TE              9     // Tearing Effect (optional, for vsync)

// Display settings
#define LCD_SPI_FREQ        40000000  // 40MHz

// =============================================================================
// Touch Controller - FT3168 (I2C Interface)
// =============================================================================
#define TOUCH_I2C_SDA       1
#define TOUCH_I2C_SCL       2
#define TOUCH_I2C_ADDR      0x38      // FT3168 I2C address
#define TOUCH_RST           -1        // Not connected (tied to LCD_RST)
#define TOUCH_INT           3         // Touch interrupt

// =============================================================================
// I/O Expander - TCA9554 (I2C Interface)
// =============================================================================
#define EXPANDER_I2C_ADDR   0x20      // TCA9554 I2C address

// Expander output pins (accent these through TCA9554)
#define EXP_PIN_LCD_BL      0         // Backlight enable (AMOLED doesn't need, but may exist)
#define EXP_PIN_LCD_RST     1         // LCD reset via expander
#define EXP_PIN_TP_RST      2         // Touch reset via expander
#define EXP_PIN_SD_CS       3         // SD card chip select

// =============================================================================
// Audio - ES8311 Codec (I2C + I2S Interface)
// =============================================================================
#define AUDIO_I2C_ADDR      0x18      // ES8311 I2C address

// I2S pins
#define I2S_BCLK            16        // Bit clock
#define I2S_LRCLK           15        // Left/Right clock (Word Select)
#define I2S_DOUT            8         // Data out to codec
#define I2S_DIN             46        // Data in from codec (microphone)
#define I2S_MCLK            -1        // Master clock (not used)

// =============================================================================
// IMU - QMI8658 6-axis (I2C Interface, shares bus with touch)
// =============================================================================
#define IMU_I2C_ADDR        0x6B      // QMI8658 I2C address

// =============================================================================
// RTC - PCF85063 (I2C Interface, shares bus with touch)
// =============================================================================
#define RTC_I2C_ADDR        0x51      // PCF85063 I2C address

// =============================================================================
// Power Management - AXP2101 (I2C Interface)
// =============================================================================
#define PMU_I2C_ADDR        0x34      // AXP2101 I2C address

// =============================================================================
// SD Card (SPI Interface)
// =============================================================================
#define SD_MOSI             11
#define SD_MISO             13
#define SD_SCLK             12
#define SD_CS               10        // Or via TCA9554 expander

// =============================================================================
// Misc
// =============================================================================
#define BOOT_PIN            0         // Boot button
#define BAT_ADC             4         // Battery voltage ADC

#endif // PINS_CONFIG_H
