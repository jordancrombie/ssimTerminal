/**
 * @file waveshare_lcd_1_85c_box.h
 * @brief Board configuration for Waveshare ESP32-S3-Touch-LCD-1.85C-BOX-EN
 *
 * Hardware: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.85C
 *
 * Display: ST77916 TFT (360x360 round) via QSPI interface
 * Touch: CST816 capacitive touch via I2C
 * Audio: PCM5101 audio decoder (I2S)
 * RTC: PCF85063 real-time clock
 * IO Expander: TCA9554PWR for GPIO expansion
 */

#ifndef WAVESHARE_LCD_1_85C_BOX_H
#define WAVESHARE_LCD_1_85C_BOX_H

// =============================================================================
// Board Identification
// =============================================================================
#define BOARD_NAME              "Waveshare ESP32-S3-Touch-LCD-1.85C-BOX"
#define DISPLAY_TYPE_QSPI       1       // QSPI display interface
#define DISPLAY_DRIVER_ST77916  1       // ST77916 TFT driver
#define TOUCH_DRIVER_CST816     1       // CST816 touch controller
#define HAS_PMU_AXP2101         0       // No AXP2101 PMU (uses MP1605GTF-Z)
#define HAS_AUDIO_ES8311        0       // No ES8311 (has PCM5101 instead)
#define HAS_AUDIO_PCM5101       1       // PCM5101 audio decoder
#define HAS_RTC_PCF85063        1       // PCF85063 RTC

// =============================================================================
// Display - ST77916 TFT (QSPI Interface)
// 360x360 round display
// =============================================================================
#define LCD_WIDTH               360
#define LCD_HEIGHT              360

// QSPI pins
#define LCD_QSPI_CS             21
#define LCD_QSPI_CLK            40
#define LCD_QSPI_D0             46      // SDA0
#define LCD_QSPI_D1             45      // SDA1
#define LCD_QSPI_D2             42      // SDA2
#define LCD_QSPI_D3             41      // SDA3

// Display control
#define LCD_TE                  18      // Tearing effect pin
#define LCD_RST_VIA_EXPANDER    1       // Reset via TCA9554 EXIO2
#define LCD_RST                 -1      // Not directly connected
#define LCD_BL                  5       // Backlight (direct GPIO, not via expander)
#define LCD_BL_VIA_EXPANDER     0       // Backlight is direct GPIO

// =============================================================================
// I2C Bus
// =============================================================================
#define I2C_SDA                 11
#define I2C_SCL                 10
#define I2C_FREQ                400000

// =============================================================================
// Touch Controller - CST816
// =============================================================================
#define TOUCH_I2C_ADDR          0x15    // CST816 I2C address (primary)
#define TOUCH_I2C_ADDR_ALT      0x5A    // CST816 alternate address
#define TOUCH_INT               4       // TP_INT
#define TOUCH_RST               -1      // Via TCA9554 EXIO1
#define TOUCH_DRIVER_CST816     1       // CST816 touch controller

// =============================================================================
// I/O Expander - TCA9554PWR
// Pin numbers are 1-based (EXIO1=1, EXIO2=2, etc.)
// Bit positions for direct register writes are (pin_num - 1)
// =============================================================================
#define EXPANDER_I2C_ADDR       0x20    // TCA9554 I2C address
#define EXP_PIN_TP_RST          0       // EXIO1 - Touch reset (bit 0)
#define EXP_PIN_LCD_RST         1       // EXIO2 - LCD reset (bit 1)
#define EXP_PIN_SD_CS           2       // EXIO3 - SD card CS (bit 2)
#define EXP_PIN_RTC_INT         3       // EXIO4 - RTC interrupt (bit 3)

// =============================================================================
// Audio - PCM5101 DAC (I2S)
// =============================================================================
#define AUDIO_I2C_ADDR          0x00    // PCM5101 has no I2C (I2S only)
#define I2S_BCLK                48      // Speak_BCK
#define I2S_LRCLK               38      // Speak_LRCK
#define I2S_DOUT                47      // Speak_DIN
#define I2S_DIN                 -1      // PCM5101 is output-only (no data in)

// Microphone (I2S input)
#define MIC_WS                  2       // MIC_WS
#define MIC_SCK                 15      // MIC_SCK
#define MIC_SD                  39      // MIC_SD

// Speaker enable (if applicable)
#define SPK_EN                  -1      // Not documented

// =============================================================================
// RTC - PCF85063
// =============================================================================
#define RTC_I2C_ADDR            0x51    // PCF85063 I2C address
#define RTC_INT                 -1      // Via TCA9554 EXIO4

// =============================================================================
// SD Card (directly wired, CS via expander)
// =============================================================================
#define SD_MOSI                 17      // SD_CMD
#define SD_MISO                 16      // SD_D0
#define SD_SCLK                 14      // SD_SCK
#define SD_CS                   -1      // Via TCA9554 EXIO3

// =============================================================================
// Power Management
// =============================================================================
// This board uses MP1605GTF-Z which has no I2C interface
#define PMU_I2C_ADDR            0x00    // Not present

// =============================================================================
// UART
// =============================================================================
#define UART_TXD                43
#define UART_RXD                44

// =============================================================================
// Buttons
// =============================================================================
#define BTN_BOOT                0

#endif // WAVESHARE_LCD_1_85C_BOX_H
