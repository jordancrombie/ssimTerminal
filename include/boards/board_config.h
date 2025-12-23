/**
 * @file board_config.h
 * @brief Board configuration selector for ssimTerminal
 *
 * This file selects the appropriate board configuration based on
 * build flags defined in platformio.ini
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// Board selection via PlatformIO build flags
#if defined(BOARD_WAVESHARE_AMOLED_1_8)
    #include "boards/waveshare_amoled_1_8.h"
#elif defined(BOARD_WAVESHARE_LCD_7)
    #include "boards/waveshare_lcd_7.h"
#elif defined(BOARD_WAVESHARE_LCD_1_85C_BOX)
    #include "boards/waveshare_lcd_1_85c_box.h"
#else
    #error "No board defined! Add -DBOARD_WAVESHARE_AMOLED_1_8, -DBOARD_WAVESHARE_LCD_7, or -DBOARD_WAVESHARE_LCD_1_85C_BOX to build_flags in platformio.ini"
#endif

// =============================================================================
// Feature flags (derived from board-specific defines)
// Use these in application code for conditional compilation
// =============================================================================

// Audio support - true if any audio hardware is present
#if HAS_AUDIO_ES8311 || HAS_AUDIO_PCM5101
    #define HAS_AUDIO 1
#else
    #define HAS_AUDIO 0
#endif

// Future feature flags can be added here:
// #define HAS_BLE (...)
// #define HAS_NFC (...)

// =============================================================================
// Common board interface validation
// =============================================================================
#ifndef LCD_WIDTH
    #error "Board config must define LCD_WIDTH"
#endif
#ifndef LCD_HEIGHT
    #error "Board config must define LCD_HEIGHT"
#endif
#ifndef I2C_SDA
    #error "Board config must define I2C_SDA"
#endif
#ifndef I2C_SCL
    #error "Board config must define I2C_SCL"
#endif

#endif // BOARD_CONFIG_H
