/**
 * @file lv_conf.h
 * @brief LVGL configuration for ssimTerminal
 *
 * Based on LVGL v8.3.x configuration template
 * Target: ESP32-S3 with 368x448 AMOLED display
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

// =============================================================================
// Color Settings
// =============================================================================
#define LV_COLOR_DEPTH          16      // 16-bit color (RGB565)
#define LV_COLOR_16_SWAP        0       // No byte swap needed for QSPI

// =============================================================================
// Memory Settings
// =============================================================================
// Use PSRAM for LVGL memory (ESP32-S3 has 8MB PSRAM)
#define LV_MEM_CUSTOM           1
#if LV_MEM_CUSTOM
    #define LV_MEM_CUSTOM_INCLUDE   <stdlib.h>
    #define LV_MEM_CUSTOM_ALLOC     malloc
    #define LV_MEM_CUSTOM_FREE      free
    #define LV_MEM_CUSTOM_REALLOC   realloc
#endif

// Memory buffer size (not used when LV_MEM_CUSTOM=1)
#define LV_MEM_SIZE             (128 * 1024)

// =============================================================================
// Display Settings
// =============================================================================
// Horizontal and vertical resolution
#define LV_HOR_RES_MAX          368
#define LV_VER_RES_MAX          448

// DPI (dots per inch) - approximate for 1.8" diagonal
#define LV_DPI_DEF              200

// Draw buffer size: 1/10 of screen for balance of RAM vs performance
// 368 * 45 * 2 bytes = ~33KB per buffer
#define LV_DISP_DEF_REFR_PERIOD 16      // ~60fps refresh
#define LV_DISP_ROT_LANDSCAPE   0       // Portrait mode

// =============================================================================
// Feature Configuration
// =============================================================================
#define LV_USE_LOG              1       // Enable logging for debug
#if LV_USE_LOG
    #define LV_LOG_LEVEL        LV_LOG_LEVEL_WARN
    #define LV_LOG_PRINTF       1       // Use printf for logging
#endif

// GPU (Software rendering only on ESP32)
#define LV_USE_GPU_STM32_DMA2D  0
#define LV_USE_GPU_NXP_PXP      0
#define LV_USE_GPU_NXP_VG_LITE  0
#define LV_USE_GPU_SWM341_DMA2D 0
#define LV_USE_GPU_ARM2D        0

// =============================================================================
// Indev Settings (Touch)
// =============================================================================
#define LV_INDEV_DEF_READ_PERIOD            30      // ms between reads
#define LV_INDEV_DEF_DRAG_LIMIT             10
#define LV_INDEV_DEF_DRAG_THROW             10
#define LV_INDEV_DEF_LONG_PRESS_TIME        400
#define LV_INDEV_DEF_LONG_PRESS_REP_TIME    100
#define LV_INDEV_DEF_GESTURE_LIMIT          50
#define LV_INDEV_DEF_GESTURE_MIN_VELOCITY   3

// =============================================================================
// Font Settings
// =============================================================================
// Built-in fonts (enable sizes we'll use)
#define LV_FONT_MONTSERRAT_8    0
#define LV_FONT_MONTSERRAT_10   0
#define LV_FONT_MONTSERRAT_12   1       // Small text
#define LV_FONT_MONTSERRAT_14   1       // Body text
#define LV_FONT_MONTSERRAT_16   1       // Body text (larger)
#define LV_FONT_MONTSERRAT_18   1       // WiFi setup titles
#define LV_FONT_MONTSERRAT_20   1       // Headings
#define LV_FONT_MONTSERRAT_22   0
#define LV_FONT_MONTSERRAT_24   1       // Large headings
#define LV_FONT_MONTSERRAT_26   0
#define LV_FONT_MONTSERRAT_28   0
#define LV_FONT_MONTSERRAT_30   0
#define LV_FONT_MONTSERRAT_32   1       // Amount display
#define LV_FONT_MONTSERRAT_34   0
#define LV_FONT_MONTSERRAT_36   0
#define LV_FONT_MONTSERRAT_38   0
#define LV_FONT_MONTSERRAT_40   0
#define LV_FONT_MONTSERRAT_42   0
#define LV_FONT_MONTSERRAT_44   0
#define LV_FONT_MONTSERRAT_46   0
#define LV_FONT_MONTSERRAT_48   1       // Large result icons

// Special fonts
#define LV_FONT_MONTSERRAT_12_SUBPX     0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SIMSUN_16_CJK           0
#define LV_FONT_UNSCII_8                0
#define LV_FONT_UNSCII_16               0

// Default font
#define LV_FONT_DEFAULT         &lv_font_montserrat_16

// Enable FreeType for custom fonts (disabled for now)
#define LV_USE_FONT_LOADER      0
#define LV_USE_FONT_FMT_TXT_LARGE 1     // Large font table support

// =============================================================================
// Text Settings
// =============================================================================
#define LV_TXT_ENC              LV_TXT_ENC_UTF8
#define LV_TXT_BREAK_CHARS      " ,.;:-_"
#define LV_TXT_LINE_BREAK_LONG_LEN 0
#define LV_TXT_COLOR_CMD        "#"

// =============================================================================
// Widget Configuration
// =============================================================================
// Core widgets (enable all we might need)
#define LV_USE_ARC              1
#define LV_USE_BAR              1
#define LV_USE_BTN              1
#define LV_USE_BTNMATRIX        1
#define LV_USE_CANVAS           1       // For QR code rendering
#define LV_USE_CHECKBOX         1
#define LV_USE_DROPDOWN         1
#define LV_USE_IMG              1
#define LV_USE_LABEL            1
#define LV_USE_LINE             1
#define LV_USE_ROLLER           1
#define LV_USE_SLIDER           1
#define LV_USE_SWITCH           1
#define LV_USE_TEXTAREA         1
#define LV_USE_TABLE            0

// Extra widgets
#define LV_USE_ANIMIMG          1       // Animated images for loading
#define LV_USE_CALENDAR         0
#define LV_USE_CHART            0
#define LV_USE_COLORWHEEL       0
#define LV_USE_IMGBTN           1
#define LV_USE_KEYBOARD         1       // For WiFi password entry
#define LV_USE_LED              1       // Status indicators
#define LV_USE_LIST             1       // WiFi network list
#define LV_USE_MENU             0
#define LV_USE_METER            0
#define LV_USE_MSGBOX           1       // Dialogs
#define LV_USE_SPAN             1
#define LV_USE_SPINBOX          0
#define LV_USE_SPINNER          1       // Loading spinner
#define LV_USE_TABVIEW          0
#define LV_USE_TILEVIEW         0
#define LV_USE_WIN              0

// =============================================================================
// Theme
// =============================================================================
#define LV_USE_THEME_DEFAULT    1
#if LV_USE_THEME_DEFAULT
    #define LV_THEME_DEFAULT_DARK       1   // Dark theme for AMOLED
    #define LV_THEME_DEFAULT_GROW       1
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif

#define LV_USE_THEME_BASIC      0
#define LV_USE_THEME_MONO       0

// =============================================================================
// Layouts
// =============================================================================
#define LV_USE_FLEX             1
#define LV_USE_GRID             1

// =============================================================================
// Other Features
// =============================================================================
// Animations
#define LV_USE_ANIMATION        1

// File system (for loading assets from SD if needed)
#define LV_USE_FS_STDIO         0
#define LV_USE_FS_POSIX         0
#define LV_USE_FS_WIN32         0
#define LV_USE_FS_FATFS         0

// PNG/JPG/GIF/BMP decoders
#define LV_USE_PNG              0
#define LV_USE_BMP              0
#define LV_USE_SJPG             0
#define LV_USE_GIF              0
#define LV_USE_QRCODE           0       // Using external QR library instead

// Snapshot
#define LV_USE_SNAPSHOT         0

// =============================================================================
// Compiler Settings
// =============================================================================
#define LV_BIG_ENDIAN_SYSTEM    0
#define LV_ATTRIBUTE_TICK_INC
#define LV_ATTRIBUTE_TIMER_HANDLER
#define LV_ATTRIBUTE_FLUSH_READY
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE 4
#define LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_LARGE_CONST
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY
#define LV_ATTRIBUTE_FAST_MEM
#define LV_ATTRIBUTE_DMA

#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning
#define LV_USE_LARGE_COORD      0

#endif // LV_CONF_H
