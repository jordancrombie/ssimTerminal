/**
 * @file waveshare_lcd_7.h
 * @brief Board configuration for Waveshare ESP32-S3-Touch-LCD-7
 *
 * Hardware: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-7
 *
 * Display: ST7262 RGB LCD (800x480) via RGB parallel interface
 * Touch: GT911 capacitive touch via I2C
 * PMU: CS8501 (no I2C interface - controlled via GPIO)
 * IO Expander: CH422G for GPIO expansion
 */

#ifndef WAVESHARE_LCD_7_H
#define WAVESHARE_LCD_7_H

// =============================================================================
// Board Identification
// =============================================================================
#define BOARD_NAME              "Waveshare ESP32-S3-Touch-LCD-7"
#define DISPLAY_TYPE_RGB        1       // RGB parallel display interface
#define DISPLAY_DRIVER_ST7262   1       // ST7262 LCD driver
#define TOUCH_DRIVER_GT911      1       // GT911 touch controller
#define HAS_PMU_AXP2101         0       // No AXP2101 PMU (has CS8501 without I2C)
#define HAS_AUDIO_ES8311        0       // No ES8311 audio codec

// =============================================================================
// Display - ST7262 RGB LCD (RGB Parallel Interface)
// Pin assignments from: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-7
// =============================================================================
#define LCD_WIDTH               800
#define LCD_HEIGHT              480

// RGB control pins
#define LCD_DE                  5       // Data Enable
#define LCD_VSYNC               3       // Vertical Sync
#define LCD_HSYNC               46      // Horizontal Sync
#define LCD_PCLK                7       // Pixel Clock

// RGB data pins - 16-bit (RGB565: 5-6-5)
// Red: R3-R7 (5 bits)
#define LCD_R3                  1
#define LCD_R4                  2
#define LCD_R5                  42
#define LCD_R6                  41
#define LCD_R7                  40

// Green: G2-G7 (6 bits)
#define LCD_G2                  39
#define LCD_G3                  0
#define LCD_G4                  45
#define LCD_G5                  48
#define LCD_G6                  47
#define LCD_G7                  21

// Blue: B3-B7 (5 bits)
#define LCD_B3                  14
#define LCD_B4                  38
#define LCD_B5                  18
#define LCD_B6                  17
#define LCD_B7                  10

// Display timing parameters (from Waveshare demo)
#define LCD_PCLK_FREQ           (16 * 1000 * 1000)  // 16MHz pixel clock
#define LCD_HSYNC_PULSE_WIDTH   4
#define LCD_HSYNC_BACK_PORCH    8
#define LCD_HSYNC_FRONT_PORCH   8
#define LCD_VSYNC_PULSE_WIDTH   4
#define LCD_VSYNC_BACK_PORCH    8
#define LCD_VSYNC_FRONT_PORCH   8

// Backlight control via CH422G expander (EXIO2)
#define LCD_BL_VIA_EXPANDER     1
#define LCD_BL                  -1      // Not directly connected

// Reset not needed for this display
#define LCD_RST_VIA_EXPANDER    0
#define LCD_RST                 -1

// =============================================================================
// I2C Bus
// =============================================================================
#define I2C_SDA                 8
#define I2C_SCL                 9
#define I2C_FREQ                400000

// =============================================================================
// Touch Controller - GT911
// =============================================================================
#define TOUCH_I2C_ADDR          0x5D    // GT911 default address (can be 0x14)
#define TOUCH_I2C_ADDR_ALT      0x14    // GT911 alternate address
#define TOUCH_INT               4       // TP_IRQ
#define TOUCH_RST               -1      // Via CH422G EXIO1

// =============================================================================
// I/O Expander - CH422G
// =============================================================================
#define EXPANDER_I2C_ADDR       0x24    // CH422G I2C address
#define EXP_PIN_TP_RST          1       // EXIO1 - Touch reset
#define EXP_PIN_LCD_BL          2       // EXIO2 - LCD backlight

// =============================================================================
// Audio - Not Available
// =============================================================================
// This board does not have ES8311 audio codec
#define AUDIO_I2C_ADDR          0x00    // Not present
#define I2S_BCLK                -1
#define I2S_LRCLK               -1
#define I2S_DOUT                -1
#define I2S_DIN                 -1
#define SPK_EN                  -1

// =============================================================================
// Power Management - CS8501 (No I2C interface)
// =============================================================================
#define PMU_I2C_ADDR            0x00    // Not present

// =============================================================================
// Other Peripherals
// =============================================================================
// SD Card
#define SD_MOSI                 11
#define SD_MISO                 13
#define SD_SCLK                 12
#define SD_CS                   10

// Buttons
#define BTN_BOOT                0

// USB
#define USB_DP                  20
#define USB_DN                  19

#endif // WAVESHARE_LCD_7_H
