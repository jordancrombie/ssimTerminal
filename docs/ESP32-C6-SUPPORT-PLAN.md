# ESP32-C6 Support Plan

## Status: BLOCKED - Requires Touch Version

The ESP32-C6-LCD-1.9 (non-touch) cannot be used as a payment terminal because touch input is required. This plan documents the implementation approach for when the **ESP32-C6-Touch-LCD-1.9** becomes available.

## Target Device

**Waveshare ESP32-C6-Touch-LCD-1.9**
- Product page: https://www.waveshare.com/esp32-c6-lcd-1.9.htm
- Wiki: https://www.waveshare.com/wiki/ESP32-C6-LCD-1.9

## Hardware Specifications

| Feature | ESP32-C6-Touch-LCD-1.9 | vs AMOLED-1.8 |
|---------|------------------------|---------------|
| Chip | ESP32-C6FH8 (RISC-V 160MHz) | ESP32-S3 (Xtensa 240MHz) |
| Architecture | RISC-V single-core | Xtensa dual-core |
| Flash | 8MB | 16MB |
| RAM | 512KB HP SRAM | 512KB SRAM + 8MB PSRAM |
| Display | ST7789V2 SPI 170x320 | SH8601 QSPI 368x448 |
| Touch | CST816 | FT3168 |
| IO Expander | TCA9554 | TCA9554 |
| PMU | None (USB/battery) | AXP2101 |
| Audio | None | ES8311 |
| IMU | QMI8658 (6-axis) | None |
| Wireless | WiFi 6, BLE 5, Zigbee/Thread | WiFi, BLE |

## Key Challenges

### 1. Different Chip Architecture
- ESP32-C6 uses RISC-V, ESP32-S3 uses Xtensa
- PlatformIO needs different platform/board configuration
- Some ESP-IDF APIs may differ

### 2. Smaller Display
- 170x320 vs 368x448 (less than half the pixels)
- QR codes must be smaller (max ~150px vs 240px)
- UI layouts need redesign for narrow 170px width
- Portrait orientation (320 height > 170 width)

### 3. SPI Display (not QSPI)
- ST7789V2 driver instead of SH8601
- Standard SPI interface (slower than QSPI)
- Arduino_GFX has ST7789 support

### 4. No PSRAM
- Only 512KB SRAM available
- LVGL buffers must be smaller
- May need to reduce color depth or use partial buffers

### 5. No Audio
- No payment result sounds
- `HAS_AUDIO` = 0 in board config

### 6. CST816 Touch (Already Supported)
- Same touch controller as LCD-1.85C-BOX
- Can reuse existing CST816 driver

## Pin Assignments (from Waveshare demo)

```cpp
// Display (SPI)
#define LCD_DC      6
#define LCD_CS      7
#define LCD_SCK     5
#define LCD_MOSI    4
#define LCD_RST     14
#define LCD_BL      15  // Backlight (TBD - verify)

// I2C (for touch, IMU, expander)
#define I2C_SDA     TBD
#define I2C_SCL     TBD

// Touch
#define TOUCH_INT   TBD
#define TOUCH_RST   TBD  // Via TCA9554?

// WS2812 RGB LED
#define WS2812_PIN  3
```

**TODO:** Get exact pin assignments from Waveshare demo code download.

## Implementation Steps

### Phase 1: PlatformIO Environment
1. Add `[env:esp32-c6-lcd-1_9]` to platformio.ini
2. Use `platform = espressif32` with ESP32-C6 board
3. Configure build flags for C6 architecture
4. Add `lib_ignore = ESP_I2S` (no audio)
5. Add `build_src_filter = -<es8311.c>` (no ES8311)

### Phase 2: Board Configuration
1. Create `include/boards/waveshare_esp32_c6_lcd_1_9.h`
2. Define all pin assignments
3. Set `HAS_AUDIO = 0`, `HAS_PMU_AXP2101 = 0`
4. Set `LCD_WIDTH = 170`, `LCD_HEIGHT = 320`
5. Define `DISPLAY_DRIVER_ST7789 = 1`

### Phase 3: Display Driver
1. Add ST7789 SPI display initialization to display.cpp
2. Use Arduino_GFX ST7789 driver
3. Configure for 170x320 resolution
4. Handle different LVGL buffer size (no PSRAM)

### Phase 4: Touch Driver
1. Reuse existing CST816 driver from LCD-1.85C-BOX
2. Verify I2C address and pin assignments
3. Test touch coordinates mapping

### Phase 5: UI Adjustments
1. Create narrow layout variants for 170px width
2. Reduce QR code size to ~140-150px
3. Adjust font sizes for smaller screen
4. Test all screens fit within 170x320

### Phase 6: LVGL Memory Optimization
1. Reduce LVGL buffer size for 512KB RAM constraint
2. Consider 1/10 screen buffer instead of 1/4
3. May need to disable some LVGL features
4. Test for memory stability

## Files to Create/Modify

| File | Action |
|------|--------|
| `platformio.ini` | Add esp32-c6-lcd-1_9 environment |
| `include/boards/waveshare_esp32_c6_lcd_1_9.h` | Create board config |
| `include/boards/board_config.h` | Add C6 board selection |
| `src/display.cpp` | Add ST7789 SPI driver |
| `include/lv_conf.h` | May need C6-specific buffer settings |
| UI screens in `src/main.cpp` | Narrow layout variants |

## Testing Checklist

- [ ] Build compiles for esp32-c6-lcd-1_9
- [ ] Upload succeeds via USB
- [ ] Display initializes (shows boot screen)
- [ ] Touch input works
- [ ] WiFi connects
- [ ] WebSocket connects to server
- [ ] QR code displays correctly (readable by phone)
- [ ] Payment flow completes
- [ ] No memory crashes under load

## Open Questions

1. **Exact pin assignments** - Need Waveshare demo download
2. **LVGL memory requirements** - May need profiling on device
3. **Display orientation** - Portrait vs landscape handling
4. **Touch calibration** - CST816 coordinate mapping for this display size

## References

- [Waveshare ESP32-C6-LCD-1.9 Wiki](https://www.waveshare.com/wiki/ESP32-C6-LCD-1.9)
- [ESP32-C6 Datasheet](https://www.espressif.com/en/products/socs/esp32-c6)
- [Arduino_GFX ST7789 Support](https://github.com/moononournation/Arduino_GFX)

---
*Created: 2025-12-23*
*Status: Planning - awaiting touch hardware*
