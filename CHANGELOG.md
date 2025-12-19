# Changelog

All notable changes to this project will be documented in this file.

## [0.1.0] - 2025-12-19

### Added
- **AXP2101 PMU driver** (`src/pmu.cpp`, `include/pmu.h`)
  - Initializes all power rails (ALDO1-4, BLDO1-2) at 3.3V for display and peripherals
  - Battery monitoring (voltage, percentage, charging status)
  - USB connection detection
  - Configures charging parameters (4.2V target, 500mA current)

- **Waveshare's modified GFX Library** (`lib/GFX_Library_for_Arduino/`)
  - Required for SH8601 AMOLED display on this hardware
  - Includes `Display_Brightness()` method for brightness control
  - Has IPS parameter support in Arduino_SH8601 constructor
  - Inherits from Arduino_TFT (not Arduino_OLED like upstream)

### Changed
- **I2C pins corrected** in `include/pins_config.h`
  - SDA: GPIO 15 (was incorrectly GPIO 8)
  - SCL: GPIO 14 (was incorrectly GPIO 9)
  - These pins match Waveshare's reference implementation

- **Display initialization** in `src/display.cpp`
  - Uses TCA9554 GPIO expander for reset sequence (P0, P1, P2)
  - Implements brightness fade-in using Waveshare's Display_Brightness() method
  - Proper QSPI bus configuration for SH8601

- **platformio.ini updates**
  - Added XPowersLib dependency for AXP2101 PMU control
  - Switched to local GFX library (Waveshare's modified version)
  - Commented out upstream moononournation/GFX library

### Fixed
- **Display not showing content** - Root cause was using upstream Arduino_GFX library
  instead of Waveshare's modified version which has correct SH8601 initialization
- **PMU power rails** - Display requires PMU to enable ALDO/BLDO rails before it will
  respond to commands
- Removed orphaned `#include "pin_config.h"` from Waveshare's Arduino_TFT.cpp

### Technical Notes
- The Waveshare ESP32-S3-Touch-AMOLED-1.8 requires specific initialization:
  1. PMU (AXP2101) must enable power rails first
  2. TCA9554 GPIO expander must toggle reset pins (P0, P1, P2)
  3. SH8601 display init via QSPI
  4. Brightness must be set explicitly (Display_Brightness)
- I2C bus has 7 devices: TCA9554 (0x20), AXP2101 (0x34), FT3168 (0x38), and others
