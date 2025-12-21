# Changelog

All notable changes to this project will be documented in this file.

## [0.5.0] - 2025-12-21

### Added
- **Settings menu** - Accessible via gear icon on idle screen
  - Brightness slider with live preview (persisted in NVS)
  - Sound effects toggle (on/off)
  - WiFi settings shortcut
  - Environment indicator
  - Terminal info (firmware version, terminal ID)
  - Save & Return button
- **Store name display** - Shows on idle screen at bottom
  - Displays store name from server (via pairing response or config_update)
  - Falls back to server hostname if no store name provided
- **Audio feedback** - Sound effects for payment results via ES8311 codec
  - Approved: Happy ascending C-E-G tones
  - Declined: Sad descending G-D tones
  - Cancelled: Single neutral beep
  - Expired: Two short beeps
  - Respects sound enabled setting
- **WebSocket stability improvements**
  - Added ping/pong heartbeat (15s interval, 3s timeout, 2 retries)
  - Connection stays alive during settings menu

### Changed
- **Production server** - Changed from `ssim.banksim.ca` to `store.regalmoose.ca`
- **State machine** - Added `SETTINGS` state
- **Heartbeat coverage** - Now sends in SETTINGS state as well

### Fixed
- **Credentials persistence** - Removed temporary NVS clear (credentials now persist across reboots)

---

## [0.4.0] - 2025-12-19

### Added
- **WiFi provisioning UI** - Full touch-based WiFi configuration
  - Scans for available networks and displays scrollable list
  - Touch keyboard for password entry
  - Stores WiFi credentials in NVS (survives reboot)
  - Rescan button to refresh network list
  - Connection failure handling with retry
- **Environment selector** - Toggle between Development and Production
  - Blue button for Development, orange for Production
  - Stored in NVS and persisted across reboots
  - Changing environment clears API credentials (requires re-pairing)
  - All server URLs configured via `ServerConfig` struct
- **Pairing flow UI** - Code entry screen for registering terminal with SSIM
  - Large text entry field for pairing codes
  - Instructions for where to get the code (SSIM Settings > Terminals)
  - Back button to return to WiFi setup
  - Environment indicator showing current server
- **Polished result screens** - Enhanced payment outcome displays
  - Circular icon backgrounds with glow effects
  - Color-coded themes: green (approved), red (declined), gray (cancelled), orange (expired/error)
  - Added support for cancelled and expired status types
- **New boot screen** - Professional branded splash screen
  - Blue circle with dollar sign ($) logo
  - "ssimTerminal" title with "Payment Terminal" tagline
  - Version number at bottom
  - Subtle glow effect on logo
  - 5-second display time with 1-second fade-out transition
  - Smooth screen transitions (renders next screen while dark to prevent flash)

### Changed
- **State machine** - Added `WIFI_SETUP` state for WiFi provisioning
- **Setup flow** - Now checks for stored WiFi credentials before connecting
  - If credentials exist: auto-connect and proceed to pairing/WebSocket
  - If no credentials: show WiFi setup screen
- **Server configuration** - Moved from hardcoded URLs to environment-based config
  - Development: `ssim-dev.banksim.ca`
  - Production: `ssim.banksim.ca`
- **Heartbeat logic** - Now sends heartbeats in all connected states (IDLE, QR_DISPLAY, RESULT)
  - Ensures server always knows terminal is alive during payment flow

### Fixed
- **Touch driver reliability** - Multiple improvements to FT3168 touch handling
  - Switched from interrupt-based to polling (GPIO 3 conflict with SD_MISO)
  - Added event flag checking (reject lift-up and reserved events)
  - Added edge filtering (5px margin to prevent phantom touches near screen edges)
  - Added separate `WIFI_PASSWORD` state to prevent accidental navigation
  - Added visible "Back" button on password screen for easier navigation

---

## [0.3.0] - 2025-12-20

### Added
- **QR code display** - Generates and displays QR codes for payment URLs
  - Uses ricmoo/QRCode library (Version 6, 41x41 modules)
  - 240x240 pixel canvas with PSRAM allocation
  - Shows amount and currency below QR code
- **payment_complete handler** - Handles payment completion from SSIM server
  - Shows "Payment Successful" (green) or "Payment Declined" (red) screen
  - Returns to idle after 3 seconds
- **Enhanced heartbeat** - Now includes terminalId, firmwareVersion, ipAddress
  - Helps SSIM server track terminal status and update device info

### Fixed
- **Terminal offline status** - Added terminalId to heartbeat for proper tracking

---

## [0.2.0] - 2025-12-19

### Added
- **WiFi connectivity** - Connects to configured network on boot
- **SSIM server pairing** - HTTP POST to `/api/terminal/pair` with device info
  - Stores API key and terminal ID in NVS (non-volatile storage)
  - Automatically uses stored credentials on subsequent boots
- **WebSocket client** - Secure connection to SSIM server (`wss://`)
  - Handles `payment_request`, `payment_result`, `payment_cancel` messages
  - Sends `heartbeat` and `payment_status` messages
  - Auto-reconnect on disconnect
- **State machine** - BOOT → CONNECTING → IDLE ⇄ QR_DISPLAY → RESULT → IDLE
- **Basic UI screens** - Boot, Connecting, Idle, Error screens with LVGL

### Changed
- **Setup flow** - Now connects WiFi first, then pairs if needed, then WebSocket
- **Boot sequence** - Shows connecting screen during WiFi/pairing phase

### Fixed
- **Boot loop crash** - WiFi must be connected before making HTTP requests
  - Previous code called `pairWithServer()` before `setupWiFi()`

---

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
