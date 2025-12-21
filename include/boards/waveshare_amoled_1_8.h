/**
 * @file waveshare_amoled_1_8.h
 * @brief Board configuration for Waveshare ESP32-S3-Touch-AMOLED-1.8
 *
 * Hardware: https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8
 *
 * Display: SH8601 AMOLED (368x448) via QSPI
 * Touch: FT3168 capacitive touch via I2C
 * PMU: AXP2101 power management via I2C
 * IO Expander: TCA9554 for reset control
 */

#ifndef WAVESHARE_AMOLED_1_8_H
#define WAVESHARE_AMOLED_1_8_H

// =============================================================================
// Board Identification
// =============================================================================
#define BOARD_NAME              "Waveshare ESP32-S3-Touch-AMOLED-1.8"
#define DISPLAY_TYPE_QSPI       1       // QSPI display interface
#define DISPLAY_DRIVER_SH8601   1       // SH8601 AMOLED driver
#define TOUCH_DRIVER_FT3168     1       // FT3168 touch controller
#define HAS_PMU_AXP2101         1       // Has AXP2101 PMU
#define HAS_AUDIO_ES8311        1       // Has ES8311 audio codec

// =============================================================================
// Display - SH8601 AMOLED (QSPI Interface)
// =============================================================================
#define LCD_WIDTH               368
#define LCD_HEIGHT              448

// QSPI pins for SH8601
#define LCD_QSPI_CLK            11
#define LCD_QSPI_D0             4
#define LCD_QSPI_D1             5
#define LCD_QSPI_D2             6
#define LCD_QSPI_D3             7
#define LCD_QSPI_CS             12
#define LCD_QSPI_FREQ           40000000  // 40MHz

// Reset via I2C expander
#define LCD_RST_VIA_EXPANDER    1
#define LCD_RST                 -1

// =============================================================================
// I2C Bus
// =============================================================================
#define I2C_SDA                 15
#define I2C_SCL                 14
#define I2C_FREQ                400000

// =============================================================================
// Touch Controller - FT3168
// =============================================================================
#define TOUCH_I2C_ADDR          0x38
#define TOUCH_INT               3
#define TOUCH_RST               -1      // Via TCA9554

// =============================================================================
// I/O Expander - TCA9554
// =============================================================================
#define EXPANDER_I2C_ADDR       0x20
#define EXP_PIN_LCD_RST         1
#define EXP_PIN_TP_RST          2
#define EXP_PIN_SD_CS           3

// =============================================================================
// Audio - ES8311 Codec
// =============================================================================
#define AUDIO_I2C_ADDR          0x18
#define I2S_BCLK                45
#define I2S_LRCLK               46
#define I2S_DOUT                40
#define I2S_DIN                 48
#define SPK_EN                  -1

// =============================================================================
// Power Management - AXP2101
// =============================================================================
#define PMU_I2C_ADDR            0x34
#define PMU_IRQ                 -1

// =============================================================================
// Other Peripherals
// =============================================================================
#define IMU_I2C_ADDR            0x6B    // QMI8658
#define RTC_I2C_ADDR            0x51    // PCF85063

// SD Card
#define SD_MOSI                 1
#define SD_MISO                 3
#define SD_SCLK                 2
#define SD_CS                   41

// Buttons
#define BTN_BOOT                0

#endif // WAVESHARE_AMOLED_1_8_H
